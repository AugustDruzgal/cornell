/**
 * V. Hunter Adams (vha3@cornell.edu)
 * PWM demo code with serial input
 * 
 * This demonstration sets a PWM duty cycle to a
 * user-specified value.
 * 
 * HARDWARE CONNECTIONS
 *   - GPIO 4 ---> PWM output
 *   - GPIO 16 ---> VGA Hsync
 *   - GPIO 17 ---> VGA Vsync
 *   - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *   - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *   - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *   - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *   - RP2040 GND ---> VGA GND
 *   - GPIO 8 ---> MPU6050 SDA
 *   - GPIO 9 ---> MPU6050 SCL
 *   - 3.3v ---> MPU6050 VCC
 *   - RP2040 GND ---> MPU6050 GND
 */

// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"

// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"

// Include custom libraries
#include "vga16_graphics_v2.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_4.h"

// // Include Chipmunk2D physics library
// #include "chipmunk/chipmunk.h"

#include "pico/time.h"
#include <stdbool.h>   // add this near the other includes


// ---------- Tunable physics knobs ----------
// Fall speed of the box (independent of "mass")
static const float BOX_GRAVITY     = 80.0f;   // pixels/s^2

// How "heavy" the box feels when it hits
static const float BOX_MASS        = 14.0f;    // larger = more torque

// How hard the board is to rotate (like moment of inertia)
static const float BOARD_INERTIA   = 1200.0f;   // larger = harder to spin

// Board spring & damping (return-to-center and friction)
static const float BOARD_SPRING_K  = 15.0f;   // was 30
static const float BOARD_DAMP      = 1.5f;    // was 5

// How strongly impact torque is scaled (first collision only)
static const float IMPACT_SCALE        = 0.08f;  // smaller = softer initial kick

// How strongly "weight" torque is scaled (while sitting)
static const float STATIC_WEIGHT_SCALE = 14.0f; // you already had something like this

// 0 < BOX_FRICTION <= 1
// 1.0  = zero friction (no slowdown along board)
// 0.9  = light friction
// 0.5  = heavy friction
static const float BOX_FRICTION   = 0.95f;   // try 0.95–0.99 for very滑滑

// Button state machine states
typedef enum _button_state {
    BUTTON_NOT_PRESSED,
    BUTTON_MAYBE_PRESSED,
    BUTTON_PRESSED,
    BUTTON_MAYBE_NOT_PRESSED
} button_state;

typedef enum {
    SUPPORT_BOARD,
    SUPPORT_BOX
} support_type_t;



// -----------------
// Simple custom physics types
// -----------------
typedef struct {
    float x, y;
    float vx, vy;
    float angle;    // radians: 0 while falling, then matches platform

    bool  on_board;    // has this box landed?
    float lx;          // x-position in *board-local* coordinates
    int   stack_level; // 0 = directly on board, 1 = on top of one box, etc.
    float vx_local;    // <--- NEW: velocity along the board

    support_type_t support_type;
    int support_index;   // which box if SUPPORT_BOX (ignored if SUPPORT_BOARD)

} phys_box_t;

typedef struct {
    float cx, cy;   // center position
    float angle;    // radians
    float omega;    // angular velocity (rad/s)
} phys_platform_t;

#define MAX_BOXES 30   

static phys_box_t boxes[MAX_BOXES];
static int num_boxes = 0;

static phys_platform_t pplat;

// spawn timer
static uint64_t last_spawn_time_us = 0;


static const float physics_dt = 1.0f / 60.0f;  // 60 Hz physics step


// ---------------------------------------------------------
// BOXES GAME parameters
// ---------------------------------------------------------
// Screen boundaries
#define SCREEN_TOP_Y 5
#define SCREEN_BOTTOM_Y 475
#define SCREEN_LEFT_X 5
#define SCREEN_RIGHT_X 635

#define SCREEN_MIDDLE_X 320

// ---- Rotating board (paddle) params ----
static float board_angle_deg = 15.0f;     // <-- control this variable to rotate the board
#define BOARD_WIDTH   300
#define BOARD_HEIGHT  12
#define BOARD_Y       (SCREEN_BOTTOM_Y - 150)   // vertical position of the board center
#define BOARD_X       ((SCREEN_LEFT_X + SCREEN_RIGHT_X)/2)  // centered horizontally
// #define MAX_BOXES 1
// the color of the box
char color = WHITE;
// Simulation parameters
#define BOX_START_X 320
#define BOX_START_Y 0
#define BOX_START_DX_MAX 1

// ---------------------------------------------------------
// BOXES GAME parameters END
// ---------------------------------------------------------


#define BUTTON_PIN 15
static button_state state = BUTTON_NOT_PRESSED;

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

// character array
char screentext[40];

// draw speed
int threshold = 10 ;




// Some macros for max/min/abs
#define min(a,b) ((a<b) ? a:b)
#define max(a,b) ((a<b) ? b:a)
#define abs(a) ((a>0) ? a:-a)

// semaphore
static struct pt_sem vga_semaphore ;

// PWM wrap value and clock divide value
// For a CPU rate of 125 MHz, this gives
// a PWM frequency of 1 kHz.
#define WRAPVAL 5000
#define CLKDIV 25.0f

// GPIO we're using for PWM
#define PWM_OUT 4

// Variable to hold PWM slice number
uint slice_num ;

// PWM duty cycle
volatile int control ;
volatile int old_control ;

float control_temp;

#define P_DEFAULT 600.0
#define I_DEFAULT 0.50
#define D_DEFAULT 100000

#define ANGLE_DEFAULT 90

#define NUM_ERRS 500

#define USEC_PER_SEC 1000000

fix15 accel_angle = int2fix15(0);
fix15 gyro_angle_delta = int2fix15(0);
fix15 complementary_angle = int2fix15(0);
fix15 angle = int2fix15(ANGLE_DEFAULT);
fix15 proportion = float2fix15(P_DEFAULT);
fix15 integral = float2fix15(I_DEFAULT);
float derivative = D_DEFAULT;
fix15 desired_angle_rad;
float error = 0.0f;

bool update_display = true;

float integral_err[NUM_ERRS] = {0};
int integral_err_index = 0;


// Randomized frop box
static float rand_unit(void) {
    return (float)rand() / (float)(RAND_MAX + 1.0f);
}


// -------------------------------------------------------------------------
// Rotated rectangle functions
// -------------------------------------------------------------------------
// ----- Rotated rectangle OUTLINE (no triangles needed) -----
static void drawRotatedRectOutline(int cx, int cy, int w, int h, float angle_deg, uint16_t color) {
    float rad = angle_deg * 3.14159265f / 180.0f;
    float c = cosf(rad), s = sinf(rad);
    float hx = 0.5f * w, hy = 0.5f * h;

    // corners: TL, TR, BR, BL
    float ux[4] = {-hx,  hx,  hx, -hx};
    float uy[4] = {-hy, -hy,  hy,  hy};
    int px[4], py[4];
    for (int i = 0; i < 4; i++) {
        float rx = ux[i]*c - uy[i]*s;
        float ry = ux[i]*s + uy[i]*c;
        px[i] = (int)(cx + rx);
        py[i] = (int)(cy + ry);
    }
    drawLine(px[0],py[0], px[1],py[1], color);
    drawLine(px[1],py[1], px[2],py[2], color);
    drawLine(px[2],py[2], px[3],py[3], color);
    drawLine(px[3],py[3], px[0],py[0], color);
}

static inline void drawBoardPaddle(void) {
    drawRotatedRectOutline(BOARD_X, BOARD_Y, BOARD_WIDTH, BOARD_HEIGHT, board_angle_deg, WHITE);
}
static void drawBoxRotated(float x, float y, float angle_rad, int size, uint16_t color) {
    float angle_deg = angle_rad * 180.0f / 3.14159265f;
    drawRotatedRectOutline((int)x, (int)y, size, size, angle_deg, color);
}

// --------------------------------------------------------------------------
// Rotated rectangle functions END
// --------------------------------------------------------------------------


// Draw the boundaries
void drawBoard() 
{
	drawVLine(SCREEN_RIGHT_X, SCREEN_TOP_Y, SCREEN_BOTTOM_Y - SCREEN_TOP_Y, WHITE);
	drawVLine(SCREEN_LEFT_X, SCREEN_TOP_Y, SCREEN_BOTTOM_Y - SCREEN_TOP_Y, WHITE);
	drawHLine(SCREEN_LEFT_X, SCREEN_BOTTOM_Y, SCREEN_RIGHT_X - SCREEN_LEFT_X, WHITE);
	drawHLine(SCREEN_LEFT_X, SCREEN_TOP_Y, SCREEN_RIGHT_X - SCREEN_LEFT_X, WHITE);

	setCursor(10, 10);
	setTextSize(1);
	setTextColor2(WHITE, BLACK);
	
	static char str[256];

	// drawRect(10, 10, 200, 100, BLACK);
	writeString(str);
}

// ------------------Initialize physics state----------------------------
static void init_box(phys_box_t *b) {
    float halfL = 0.5f * (float)BOARD_WIDTH;  // half-length of board
    float r     = 10.0f;                      // half-size of box (20x20)

    float u = rand_unit(); // random in [0,1)

    float minX = pplat.cx - halfL + r;
    float maxX = pplat.cx + halfL - r;

    b->x     = minX + u * (maxX - minX);  // random x "above" the board region

    // b->x     = 280.0f;          // TEST: stacking in the middle
    b->y     = (float)BOX_START_Y;        // from the top
    b->vx    = 0.0f;
    b->vy    = 0.0f;
    b->angle = 0.0f;                      // horizontal while falling

    b->on_board    = false;
    b->lx          = 0.0f;
    b->stack_level = 0;

    b->vx_local = 0.0f;    // <--- NEW: velocity along the board
     // by default, not actually supported by anything yet
    b->support_type  = SUPPORT_BOARD;
    b->support_index = -1;
}


static void physics_init(void) {
    // Platform
    pplat.cx    = (float)BOARD_X;
    pplat.cy    = (float)BOARD_Y;
    pplat.angle = board_angle_deg * 3.14159265f / 180.0f; // convert deg -> rad
    pplat.omega = 0.0f;

    // First box
    num_boxes = 0;
    init_box(&boxes[num_boxes++]);

    // Spawn timer: "now"
    last_spawn_time_us = time_us_64();
}


static void physics_step(void) {
    // --- Constants (tweak to taste) ---
    const float g           = BOX_GRAVITY;
    const float floor_y     = (float)SCREEN_BOTTOM_Y - 2.0f;
    const float L           = (float)BOARD_WIDTH;
    const float H           = (float)BOARD_HEIGHT;
    const float r           = 10.0f;          // half-size of box (20x20)
    const float spring_k    = BOARD_SPRING_K;
    const float damp        = BOARD_DAMP;
    const float restitution = 0.0f;
    const float STACK_BIN   = 1.2f * r;       // x-range on board considered "same column"

    float halfL = 0.5f * L;
    float halfH = 0.5f * H;

    float support_half_width;
    float box_half_width = r;  // since box width = 2*r



    // 0) Spawn new box every 5 seconds
    const uint64_t SPAWN_INTERVAL_US = 1000000; // 1 second
    uint64_t now = time_us_64();
    if (num_boxes < MAX_BOXES && (now - last_spawn_time_us) >= SPAWN_INTERVAL_US) {
        init_box(&boxes[num_boxes++]);
        last_spawn_time_us = now;
    }

    // Precompute board trig
    float c = cosf(pplat.angle);
    float s = sinf(pplat.angle);

    // 1) Update boxes
    for (int i = 0; i < num_boxes; i++) {
        phys_box_t *b = &boxes[i];

        // ----------------------------
        // A) Boxes that are already on the board: ride with the board
        // ----------------------------
        // if (b->on_board) {
        //     // Its local y = board surface + stack height
        //     float ly = -halfH - r - 2.0f * r * (float)b->stack_level;

        //     float wx = b->lx * c - ly * s;
        //     float wy = b->lx * s + ly * c;

        //     b->x = pplat.cx + wx;
        //     b->y = pplat.cy + wy;
        //     b->angle = pplat.angle;
        //     // No gravity integration while stuck to board
        //     continue;
        // }
            // ----------------------------
            // A) Boxes that are already on some support: can slide along board
            // ----------------------------
            if (b->on_board) {
                   // 0) If I'm standing on another box, but that box is no longer on the board,
                    //    I should also start falling.
                    if (b->support_type == SUPPORT_BOX) {
                        int si = b->support_index;
                        if (si < 0 || !boxes[si].on_board) {
                            // Convert from local coordinates back to world before dropping
                            float ly = -halfH - r - 2.0f * r * (float)b->stack_level;

                            float wx = b->lx * c - ly * s;
                            float wy = b->lx * s + ly * c;
                            b->x = pplat.cx + wx;
                            b->y = pplat.cy + wy;

                            // Turn tangential velocity into world vx,vy as we let it go
                            b->vx = b->vx_local * c;
                            b->vy = b->vx_local * s;

                            b->on_board      = false;
                            b->support_type  = SUPPORT_BOARD;
                            b->support_index = -1;

                            // Done with this box for this step
                            continue;
                        }
                    }
                // Gravity component along the board (sign may need flipping
                // depending on what "positive" angle means visually)
                float a_t = g * sinf(pplat.angle);
                // If this makes boxes slide uphill, change to: float a_t = -g * sinf(pplat.angle);

                // Update tangential velocity along the board
                b->vx_local += a_t * physics_dt;

                // Apply friction
                b->vx_local *= BOX_FRICTION;

                // Move along the support
                b->lx += b->vx_local * physics_dt;

                // Height of the stack this box sits on (same formula as before)
                float ly = -halfH - r - 2.0f * r * (float)b->stack_level;

                // --- Decide how wide the support is and where its center is ---
                float box_half_width = r;    // half box width
                float support_half_width;
                float support_center_lx;

                if (b->support_type == SUPPORT_BOARD) {
                    // Board: wide support, centered at lx = 0
                    support_half_width = halfL;
                    support_center_lx  = 0.0f;
                } else { // SUPPORT_BOX
                    // Standing on a single box:
                    //  - support width = box width
                    //  - support center = that box's lx
                    support_half_width = box_half_width;
                    if (b->support_index >= 0) {
                        support_center_lx = boxes[b->support_index].lx;
                    } else {
                        // Failsafe: treat as board center if index missing
                        support_center_lx = 0.0f;
                    }
                }

                // Drop when this box's CENTER is no longer above its support's footprint
                // i.e., when distance from support center exceeds (support_half - own_half)
                float rel_lx = b->lx - support_center_lx;
                if (fabsf(rel_lx) > (support_half_width - box_half_width)) {
                    // Convert local pos back to world before we let it fall
                    float wx = b->lx * c - ly * s;
                    float wy = b->lx * s + ly * c;
                    b->x = pplat.cx + wx;
                    b->y = pplat.cy + wy;

                    // Local → world velocity (only tangential, no jump in normal)
                    b->vx = b->vx_local * c;
                    b->vy = b->vx_local * s;

                    // It is no longer stuck to anything
                    b->on_board      = false;
                    b->support_type  = SUPPORT_BOARD;
                    b->support_index = -1;
                    continue;
                }


                // Still supported: convert local coords back to world
                float wx = b->lx * c - ly * s;
                float wy = b->lx * s + ly * c;
                b->x = pplat.cx + wx;
                b->y = pplat.cy + wy;
                b->angle = pplat.angle;

                continue;
            }




        // ----------------------------
        // B) Falling boxes
        // ----------------------------

        // Gravity integration
        b->vy += g * physics_dt;
        b->x  += b->vx * physics_dt;
        b->y  += b->vy * physics_dt;

        // Floor collision (if it ever gets that low)
        if (b->y + r > floor_y) {
            b->y = floor_y - r;
            b->vy *= -restitution;
        }

        // --- Convert to board-local coordinates ---
        float dx = b->x - pplat.cx;
        float dy = b->y - pplat.cy;

        float lx =  dx * c + dy * s;   // along the board
        float ly = -dx * s + dy * c;   // normal to the board (negative = above)

        // Is it horizontally above the board?
        bool over_board = (fabsf(lx) <= halfL + r);

        // Determine current stack height in this column
        int column_height   = 0;   // # of boxes already stacked here
        int top_box_index   = -1;  // which box is at the top in this column

        for (int j = 0; j < num_boxes; j++) {
            if (j == i) continue;
            if (!boxes[j].on_board) continue;

            if (fabsf(boxes[j].lx - lx) < STACK_BIN) {
                // stack_level = 0 means directly on board
                int candidate_height = boxes[j].stack_level + 1;
                if (candidate_height > column_height) {
                    column_height = candidate_height;
                    top_box_index = j;
                }
            }
        }


                float ly_contact = -halfH - r - 2.0f * r * (float)column_height;

        // Did we cross that surface while moving downward?
        bool crossing_surface = (ly > ly_contact);
        if (over_board && crossing_surface && (b->vy > 0.0f)) {
            float vy_before = b->vy;

            // // Snap onto top of this column
            // b->on_board    = true;
            // b->stack_level = column_height;
            // b->lx          = lx;
            // b->vx_local    = 0.0f;   // start with no sliding

            b->on_board    = true;
            b->stack_level = column_height;
            b->lx          = lx;

            // small random tangential speed so stacks don't remain perfectly rigid
            float jitter = (rand_unit() - 0.5f) * 5.0f;   // between -2.5 and +2.5 (tune)
            b->vx_local    = jitter;


            // Decide what we're standing on:
            if (column_height == 0) {
                // directly on the board
                b->support_type  = SUPPORT_BOARD;
                b->support_index = -1;
            } else {
                // on top of another box
                b->support_type  = SUPPORT_BOX;
                b->support_index = top_box_index;
            }

            float ly_new = ly_contact;

            float wx = lx * c - ly_new * s;
            float wy = lx * s + ly_new * c;

            b->x = pplat.cx + wx;
            b->y = pplat.cy + wy;

            // Stick to support: zero world velocities for now
            b->vx    = 0.0f;
            b->vy    = 0.0f;
            b->angle = pplat.angle;



            // Impact torque on board (same idea as your old code)
            float hit_pos        = lx / halfL;   // -1..+1 along the board
            float impact_speed   = fabsf(vy_before);
            float torque_impulse = BOX_MASS * impact_speed * hit_pos * IMPACT_SCALE;
            float delta_omega    = torque_impulse / BOARD_INERTIA;
            pplat.omega         += delta_omega;
        }
    }

    // 2) Continuous weight torque from all boxes sitting on the board
    for (int i = 0; i < num_boxes; i++) {
        if (!boxes[i].on_board) continue;

        float lx = boxes[i].lx;  // lever arm along the board
        float weight_torque = BOX_MASS * g * (lx / halfL) * STATIC_WEIGHT_SCALE;
        float d_omega       = (weight_torque / BOARD_INERTIA) * physics_dt;
        pplat.omega        += d_omega;
    }

    // 3) Board torsion spring around angle = 0 (return to center)
    float torque = -spring_k * pplat.angle - damp * pplat.omega;
    pplat.omega += torque * physics_dt;
    pplat.angle += pplat.omega * physics_dt;

    board_angle_deg = pplat.angle * 180.0f / 3.14159265f;
}


// ------------------End initialize physics state----------------------------

// Thread that draws to VGA display
static PT_THREAD (protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

    physics_init();

    // We will start drawing at column 81
    static int xcoord = 81 ;
    
    // Rescale the measurements for display
    static float OldRange = 500. ; // (+/- 250)
    static float NewRange = 150. ; // (looks nice on VGA)
    static float OldMin = -250. ;
    static float OldMax = 250. ;

    // Control rate of drawing
    static int throttle ;

    // Draw the static aspects of the display
    setTextSize(1) ;
    setTextColor2(WHITE, BLACK);


    // spawnbox();
    // Draw background + static board ONCE
    // fillRect(0, 0, 640, 480, BLACK);

    // drawBoard();

    

    while (true) {
        
        // Step custom physics world
        physics_step();

        fillRect(0, 0, 640, 480, BLACK);

        // fillRect(0, 0, 640, 480, BLACK);

        drawBoardPaddle();
        // Draw the falling box from custom physics
        int box_size = 20;
        for (int i = 0; i < num_boxes; i++) {
            drawBoxRotated(boxes[i].x, boxes[i].y, boxes[i].angle, box_size, WHITE);
        }

        // ~60 fps
        // drawRect(300, 220, 40, 40, WHITE);
        
        // Simple frame pacing ~60 fps (16 ms) or others
        PT_YIELD_usec(16000);

        // // Wait on semaphore
        // PT_SEM_WAIT(pt, &vga_semaphore);
           
        //}
    }

    

    // Indicate end of thread
    PT_END(pt);
}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_vga) ;
    // pt_add_thread(protothread_button) ;
    pt_schedule_start ;
}

int main() {

    // Overclock
    set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();

    // Seed PRNG once so rand() is different each boot
    srand((unsigned int) time_us_32());

    printf("Starting\n");

    // Initialize VGA
    initVGA() ;

    // ////////////////////////////////////////////////////////////////////////
    // ///////////////////////// I2C CONFIGURATION ////////////////////////////
    // i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    // gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    // gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;
    
	// gpio_set_dir(BUTTON_PIN, false);
	// gpio_pull_up(BUTTON_PIN);

    // // Pullup resistors on breakout board, don't need to turn on internals
    // // gpio_pull_up(SDA_PIN) ;
    // // gpio_pull_up(SCL_PIN) ;

    // // MPU6050 initialization
    // mpu6050_reset();
    // mpu6050_read_raw(acceleration, gyro);

    // ////////////////////////////////////////////////////////////////////////
    // ///////////////////////// PWM CONFIGURATION ////////////////////////////
    // ////////////////////////////////////////////////////////////////////////
    // // Tell GPIO PWM_OUT that it is allocated to the PWM
    // gpio_set_function(5, GPIO_FUNC_PWM);
    // // gpio_set_function(4, GPIO_FUNC_PWM);
    // gpio_set_function(PWM_OUT, GPIO_FUNC_PWM);

    // // Find out which PWM slice is connected to GPIO PWM_OUT (it's slice 2)
    // slice_num = pwm_gpio_to_slice_num(PWM_OUT);

    // // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // // and register our interrupt handler
    // pwm_clear_irq(slice_num);
    // pwm_set_irq_enabled(slice_num, true);
    // irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    // irq_set_enabled(PWM_IRQ_WRAP, true);

    // // This section configures the period of the PWM signals
    // pwm_set_wrap(slice_num, WRAPVAL) ;
    // pwm_set_clkdiv(slice_num, CLKDIV) ;

    // // This sets duty cycle
    // pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    // pwm_set_chan_level(slice_num, PWM_CHAN_A, 3125);

    // // Start the channel
    // pwm_set_mask_enabled((1u << slice_num));

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // pt_add_thread(protothread_serial) ;
    pt_schedule_start ;

}
