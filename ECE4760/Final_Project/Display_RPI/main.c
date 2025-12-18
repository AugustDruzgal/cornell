/** ARCHIVE: 12.05.2025 WORKING VERSION!!! Editing the gaming elements
 * Rotating Board + Sliding Grid-based Boxes
 *
 * Boxes fall straight down under gravity in world coordinates.
 * When they hit the inclined board, they snap into an invisible
 * grid attached to the board that can slide along it. Boxes on the 
 * grid uses the board coordinates.
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"

// VGA / protothreads
#include "vga16_graphics_v2.h"
// #include "pt_cornell_rp2040_v1_4_client.h"
#include "client.c"

// ---------------------------------------------------------
// 0. GLOBAL CONSTANTS & STRUCTS
// ---------------------------------------------------------

// Screen boundaries
#define SCREEN_TOP_Y    5
#define SCREEN_BOTTOM_Y 475
#define SCREEN_LEFT_X   5
#define SCREEN_RIGHT_X  635

#define SCREEN_CENTER_Y (SCREEN_TOP_Y + SCREEN_BOTTOM_Y)/2
#define SCREEN_CENTER_X (SCREEN_LEFT_X + SCREEN_RIGHT_X)/2

// Board geometry
#define BOARD_WIDTH   300
#define BOARD_HEIGHT  12
#define BOARD_Y       (SCREEN_BOTTOM_Y - 150)
#define BOARD_X       ((SCREEN_LEFT_X + SCREEN_RIGHT_X) * 0.5f)

// Box / grid cell size
#define BOX_SIZE_PIX  20
#define BOX_HALF      (BOX_SIZE_PIX * 0.5f)

// Grid dimensions (along the board and upward)
#define GRID_COLS 15   // (BOARD_WIDTH / BOX_SIZE_PIX) = 300 / 20 = 15
#define GRID_ROWS 12                             // tweakable

#define GAME_OVER_TIME 5
#define GAME_OVER_TIME_US GAME_OVER_TIME * 1000000

#define TITLE "Block Stacker"
#define TITLE_DRIFT_MAX_X 10
#define TITLE_DRIFT_MAX_Y 5
#define TITLE_DRIFT_RATE 0.05
#define TITLE_TEXT_SIZE 5
#define TITLE_COLOR YELLOW

#define YOU_LOSE "YOU LOSE :("
#define YOU_LOSE_DRIFT_MAX_X 10
#define YOU_LOSE_DRIFT_MAX_Y 5
#define YOU_LOSE_DRIFT_RATE 0.05
#define YOU_LOSE_TEXT_SIZE 5
#define YOU_LOSE_COLOR PINK

#define GREETING_PHRASE_1 "The best game you will ever play!"
#define GREETING_PHRASE_2 "What a time to be alive!"
#define GREETING_PHRASE_3 "Time to stack those blocks!"
#define GREETING_PHRASE_4 "Lookin' good today!"
#define GREETING_PHRASE_5 "You are so cool!"
#define GREETING_DRIFT_MAX_X 20
#define GREETING_DRIFT_MAX_Y 5
#define GREETING_DRIFT_RATE 0.08
#define GREETING_TEXT_SIZE 5
#define GREETING_COLOR DARK_ORANGE

#define LOSING_PHRASE_1 "Have you tried getting good?"
#define LOSING_PHRASE_2 "Toddlers can stack blocks, why can't you?"
#define LOSING_PHRASE_3 "How are you so bad at this?"
#define LOSING_PHRASE_4 "Do better next time!"
#define LOSING_PHRASE_5 "That was super embarrassing"
#define LOSING_DRIFT_MAX_X 20
#define LOSING_DRIFT_MAX_Y 5
#define LOSING_DRIFT_RATE 0.08
#define LOSING_TEXT_SIZE 5

#define CONTROLLER_PHRASE_1 "Connect your controller!"
#define CONTROLLER_PHRASE_1_COLOR RED
#define CONTROLLER_PHRASE_2 "Controller Connected"
#define CONTROLLER_PHRASE_2_COLOR GREEN
#define CONTROLLER_PHRASE_3 "Pick up your controller to start!"
#define CONTROLLER_PHRASE_3_COLOR LIGHT_BLUE
#define CONTROLLER_DRIFT_MAX_X 10
#define CONTROLLER_DRIFT_MAX_Y 5
#define CONTROLLER_DRIFT_RATE 0.03

#define TIME_PHRASE_COLOR LIGHT_BLUE
#define TIME_DRIFT_MAX_X 30
#define TIME_DRIFT_MAX_Y 5
#define TIME_DRIFT_RATE 0.1

#define START_DELAY_MAX 5.0
#define START_DELAY_MAX_US (uint64_t) (START_DELAY_MAX * 1000000)

// Physics constants - not all used in the final version
static const float BOX_GRAVITY         = 80.0f;   // pixels/s^2, downward (screen)
static const float BOX_MASS            = 12.0f;
static const float BOARD_INERTIA       = 1200.0f;
static const float BOARD_SPRING_K      = 15.0f;
static const float BOARD_DAMP          = 1.5f;
static const float STATIC_WEIGHT_SCALE = 14.0f;

// Grid sliding parameters - not all used in the final version
static const float GRID_SLIDE_SCALE = 1.0f;   // how strongly gravity along board moves pile
static const float GRID_FRICTION    = 0.99f;  // 0<GRID_FRICTION<=1, lower = more damping

// Time step (matches PT_YIELD_usec(16000))
static const float PHYSICS_DT = 0.016f;

// The angle of the device
static float tilt_angle = 0.00f;

static float c = 0;
static float s = 0;

static bool title_screen = false;
static bool game_screen = false;
static bool end_screen = false;

// TODO: Add start delay after picking up the controller
static uint64_t start_delay = START_DELAY_MAX_US;

// Spawn timing (changeable)
static const uint64_t SPAWN_INTERVAL_US = 1000000;  // 1 second

// Simple platform struct
typedef struct {
    float cx, cy;     // center position (world coordinates)
    float angle;      
    float omega;      // angular velocity (rad/s)
} phys_platform_t;

// Falling box 
typedef struct {
    float x, y;       // world coordinates (screen pixels)
    float vy;         
    bool  active;
    uint16_t color;
} falling_box_t;

// Spinning decoration block
typedef struct _display_block{
    float x, y;
    float max_x_drift;
    float max_y_drift;
    float drift_rate;
    float spin_rate;
    uint16_t color;
    uint16_t size;
} display_block;

// ---------------------------------------------------------
// 1. ADDED PARAMETERS (as the code updated)
// ---------------------------------------------------------

// Board
static phys_platform_t pplat;

typedef struct _grid_location
{
    bool occupied;
    uint16_t color;
} grid_location;

// Grid occupancy: true = there is a box in that cell
static grid_location grid[GRID_COLS][GRID_ROWS];

static char str_buf[64];
static char *greeting_phrase;
static char *losing_phrase;

static float grid_offset_u = 0.0f;  // residual offset in [-BOX_SIZE_PIX, BOX_SIZE_PIX]
static float grid_v        = 0.0f;  // velocity along board (u/s)

// Falling boxes pool - not all used in the final version
#define MAX_FALLING 4
static falling_box_t falling[MAX_FALLING];

// Timing for spawns
static uint64_t last_spawn_time_us = 0;

// For drawing
static float board_angle_deg = 15.0f; // initial angle in degrees

// Precompute once
#define INV_RANDMAX (1.0f / (float)(RAND_MAX))
#define MICROSECONDS (1.0f / 1000000.0f)
#define DEGtoRAD (3.14159265f / 180.0f)

// --------------------------
// GAME STATES
// --------------------------
#define MAX_HEALTH_POINTS 5
static int         health_points     = MAX_HEALTH_POINTS;
static bool        game_over         = false;
static uint64_t    game_start_us     = 0;
static float       final_time_s      = 0.0f;

// ---------------------------------------------------------
// 2. HELPER FUNCTIONS FOR DRAWING
// ---------------------------------------------------------

// Random in [0,1)
static float rand_unit(void) {
    return (float)rand() * INV_RANDMAX;
}

// Rotated rectangle outline
static void drawRotatedRectOutline(int cx, int cy, int w, int h,
                                   float c, float s, uint16_t color)
{
    float hx = 0.5f * w, hy = 0.5f * h;

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

static inline void hideBoardPaddle(float c, float s) {
    drawRotatedRectOutline(BOARD_X, BOARD_Y, BOARD_WIDTH, BOARD_HEIGHT,
                           c, s, BLACK);
}

static inline void drawBoardPaddle(float c, float s) {
    drawRotatedRectOutline(BOARD_X, BOARD_Y, BOARD_WIDTH, BOARD_HEIGHT,
                           c, s, LIGHT_BLUE);
}

static void drawBoxRotated(float x, float y, float c, float s,
                           int size, uint16_t color)
{
    // float angle_deg = angle_rad * 180.0f / 3.14159265f;
    drawRotatedRectOutline((int)x, (int)y, size, size, c, s, color);
}

// Optional: draw screen borders. Not used in final version.
static void drawBoardBounds(void) {
    drawVLine(SCREEN_RIGHT_X, SCREEN_TOP_Y,
              SCREEN_BOTTOM_Y - SCREEN_TOP_Y, WHITE);
    drawVLine(SCREEN_LEFT_X,  SCREEN_TOP_Y,
              SCREEN_BOTTOM_Y - SCREEN_TOP_Y, WHITE);
    drawHLine(SCREEN_LEFT_X,  SCREEN_BOTTOM_Y,
              SCREEN_RIGHT_X - SCREEN_LEFT_X, WHITE);
    drawHLine(SCREEN_LEFT_X,  SCREEN_TOP_Y,
              SCREEN_RIGHT_X - SCREEN_LEFT_X, WHITE);
}

// ---------------------------------------------------------
// 3. CONVERSION BETWEEN GRID AND WORLD COORDINATES
// ---------------------------------------------------------

/**
 * 
 * Board-local coordinates:
 * u: along the board
 * v: normal to the board (negative = above board center)
 */
static void grid_cell_world_center(int col, int row,
                                   float *out_x, float *out_y,
                                   float c, float s,
                                   float halfL, float halfH)
{
    // Base u of col 0 center (no sliding) is -halfL + BOX_HALF.
    float base_u0 = -halfL + BOX_HALF;

    // Local "along-board" coordinate u for this column center
    float u = base_u0 + grid_offset_u + (float)col * BOX_SIZE_PIX;

    // Local "normal" coordinate v (negative is above the board center)
    float v = -halfH - BOX_HALF - (float)row * BOX_SIZE_PIX;

    // Local to world coordinates
    float dx = u * c - v * s;
    float dy = u * s + v * c;

    *out_x = pplat.cx + dx;
    *out_y = pplat.cy + dy;
}

// Forward declaration?
static void remove_grid_cell(int col, int row);

/**
 * Remove one occupied grid cell and apply health damage.
 */
static void remove_grid_cell(int col, int row)
{
    if (!grid[col][row].occupied) return;  // already empty

    grid[col][row].occupied = false;

    if (health_points > 0) {
        health_points--;
        if (health_points == 0 && !game_over) {
            game_over = true;
            uint64_t now = time_us_64();
            final_time_s = (now - game_start_us) * MICROSECONDS;
        }
    }
}

uint16_t rand_color(void)
{
    uint16_t rand_color = rand()%6;

    switch (rand_color)
    {
        case 0:
            rand_color = BLUE;
            break;
        case 1:
            rand_color = RED;
            break;
        case 2:
            rand_color = PINK;
            break;
        case 3:
            rand_color = GREEN;
            break;
        case 4:
            rand_color = YELLOW;
            break;
        case 5:
            rand_color = DARK_ORANGE;
            break;
    }

    return rand_color;
}


// Shift the entire grid array by 'shift' columns.
// Positive shift: pile moves toward +u (right in board-local),
// negative shift: toward -u (left).
// Columns that move past the edge are dropped, and cost health points.
static void shift_grid(int shift) {
    if (shift == 0) return;

    grid_location new_grid[GRID_COLS][GRID_ROWS];

    // Clear new_grid
    for (int col = 0; col < GRID_COLS; col++) {
        for (int row = 0; row < GRID_ROWS; row++) {
            new_grid[col][row].occupied = false;
        }
    }

    // Move old cells into new positions
    for (int src = 0; src < GRID_COLS; src++) {
        for (int row = 0; row < GRID_ROWS; row++) {
            if (!grid[src][row].occupied) continue;

            int dst = src + shift;

            if (dst < 0 || dst >= GRID_COLS) {
                // This cell slides off the board edge -> remove & damage HP
                remove_grid_cell(src, row);
            } else {
                new_grid[dst][row].occupied = true;
                new_grid[dst][row].color = grid[src][row].color;
            }
        }
    }

    // Copy back
    memcpy(grid, new_grid, sizeof(grid));
}

// ---------------------------------------------------------
// 4. PHYSICS INITIALIZATION
// ---------------------------------------------------------

static void clear_grid(void) {
    for (int i = 0; i < GRID_COLS; i++) {
        for (int j = 0; j < GRID_ROWS; j++) {
            grid[i][j].occupied = false;
        }
    }
}

static void clear_falling(void) {
    for (int i = 0; i < MAX_FALLING; i++) {
        falling[i].active = false;
    }
}

static void physics_init(void) {
    // Platform
    pplat.cx    = (float)BOARD_X;
    pplat.cy    = (float)BOARD_Y;
    pplat.angle = board_angle_deg * DEGtoRAD; // convert deg -> rad ; DEGtoRAD = 3.14159265f / 180.0f
    pplat.omega = 0.0f;

    grid_offset_u = 0.0f;
    grid_v        = 0.0f;

    clear_grid();
    clear_falling();

    last_spawn_time_us = time_us_64();

    // --- GAME INITIATION STATES ---
    health_points = MAX_HEALTH_POINTS;
    game_over     = false;
    game_start_us = last_spawn_time_us;
    final_time_s  = 0.0f;
}


// Spawn a single new falling box at a random world x
static void spawn_falling_box(void) {
    // Find an inactive slot
    int idx = -1;
    for (int i = 0; i < MAX_FALLING; i++) {
        if (!falling[i].active) {
            idx = i;
            break;
        }
    }
    if (idx < 0) return; // no free slot

    float halfL = 0.5f * (float)BOARD_WIDTH;

    // World x range above the board
    float x_min = (float)BOARD_X - halfL + BOX_HALF;
    float x_max = (float)BOARD_X + halfL - BOX_HALF;

    float u = rand_unit();
    float x = x_min + u * (x_max - x_min);

    // Spawn near top of the screen
    falling[idx].x      = x;
    falling[idx].y      = (float)SCREEN_TOP_Y + BOX_HALF + 10.0f;
    falling[idx].vy     = 0.0f;
    falling[idx].active = true;
    falling[idx].color  = rand_color();
}

// Remove grid cells that are off-screen
static void cleanup_grid(float c, float s) {
    // float c     = cosf(pplat.angle); // This would be redundant
    // float s     = sinf(pplat.angle);
    float halfL = 0.5f * (float)BOARD_WIDTH;
    float halfH = 0.5f * (float)BOARD_HEIGHT;

    (void)halfL; // not strictly needed here but kept for clarity

    for (int col = 0; col < GRID_COLS; col++) {
        for (int row = 0; row < GRID_ROWS; row++) {
            if (!grid[col][row].occupied) continue;

            float x, y;
            grid_cell_world_center(col, row, &x, &y, c, s, halfL, halfH);

            if (x < (SCREEN_LEFT_X  - BOX_SIZE_PIX) ||
                x > (SCREEN_RIGHT_X + BOX_SIZE_PIX) ||
                y > (SCREEN_BOTTOM_Y + BOX_SIZE_PIX))
            {
                // Box leaves the visible world -> remove & damage HP
                remove_grid_cell(col, row);
            }
        }
    }
}


// ---------------------------------------------------------
// 5. MAIN PHYSICS ITERATIONS
// ---------------------------------------------------------

static void physics_step(void) {
    if (game_over) {
        // Freeze physics when game is over
        return;
    }
    // 0) Spawn timer
    uint64_t now_us = time_us_64();
    if (now_us - last_spawn_time_us >= SPAWN_INTERVAL_US) {
        spawn_falling_box();
        last_spawn_time_us = now_us;
    }

    // Board trig/cartesian helpers
    // float c     = cosf(pplat.angle);
    // float s     = sinf(pplat.angle);
    float halfL = 0.5f * (float)BOARD_WIDTH;
    float halfH = 0.5f * (float)BOARD_HEIGHT;

    // 1) UPDATE GRID SLIDING
    // Gravity along board: component of downward gravity projected onto board axis
    float a_t = BOX_GRAVITY * sinf(pplat.angle);          // acceleration along board (u)
    grid_v  += a_t * PHYSICS_DT * GRID_SLIDE_SCALE;       // scaled 
    grid_v  *= GRID_FRICTION;
    grid_offset_u += grid_v * PHYSICS_DT;

    // Convert large offset into whole-column shifts
    int shift = 0;
    while (grid_offset_u >= BOX_SIZE_PIX) {
        grid_offset_u -= BOX_SIZE_PIX;
        shift += 1;
    }
    while (grid_offset_u <= -BOX_SIZE_PIX) {
        grid_offset_u += BOX_SIZE_PIX;
        shift -= 1;
    }
    if (shift != 0) {
        shift_grid(shift);   // this drops boxes that slide off the board edge
    }

    // 2) UPDATE FALLING BOXES (landing logic)
    for (int i = 0; i < MAX_FALLING; i++) {
        if (!falling[i].active) continue;

        falling_box_t *fb = &falling[i];

        // Gravity in world coordinates 
        fb->vy += BOX_GRAVITY * PHYSICS_DT;
        fb->y  += fb->vy * PHYSICS_DT;

        // If it goes below screen bottom, delete it
        if (fb->y - BOX_HALF > SCREEN_BOTTOM_Y) {
            fb->active = false;
            continue;
        }

        // Transform current world position to board coordinates
        float dx = fb->x - pplat.cx;
        float dy = fb->y - pplat.cy;

        float u =  dx * c + dy * s;   // along board
        float v = -dx * s + dy * c;   // normal (negative = above board center)

        // If not horizontally over the board in board coords, let it fall
        if (fabsf(u) > halfL + BOX_HALF) {
            continue;
        }

        // Determine local column index from current sliding
        float base_u0 = -halfL + BOX_HALF;
        float u_col0  = base_u0 + grid_offset_u;

        float u_rel   = u - u_col0;
        float col_f   = u_rel / BOX_SIZE_PIX;           // fractional column index
        int   col     = (int)floorf(col_f + 0.5f);      // nearest integer column

        // Clamp column so that boxes over the board always get a valid column
        if (col < 0) col = 0;
        if (col >= GRID_COLS) col = GRID_COLS - 1;

        // Find first valid empty row in that column
        // row == 0 OR cell below is occupied.
        int target_row = -1;
        for (int row = 0; row < GRID_ROWS; row++) {
            if (!grid[col][row].occupied) {
                if (row == 0 || grid[col][row - 1].occupied) {
                    target_row = row;
                    break;
                }
            }
        }

        if (target_row < 0) {
            continue;
        }

        // Board coordinates for the center of the target cell
        float v_cell = -halfH - BOX_HALF - (float)target_row * BOX_SIZE_PIX;

        // LANDING CONDITION in board coordinates
        if (v >= v_cell) {
            grid[col][target_row].occupied = true;
            grid[col][target_row].color = falling[i].color;
            fb->active = false;
        }
    }

    // 3) BOARD TORQUE FROM BOXES IN THE GRID (needing tilt feedback)
    float g = BOX_GRAVITY;

    for (int col = 0; col < GRID_COLS; col++) {
        for (int row = 0; row < GRID_ROWS; row++) {
            if (!grid[col][row].occupied) continue;

            // Lever arm along the board
            float base_u0 = -halfL + BOX_HALF;
            float u = base_u0 + grid_offset_u + (float)col * BOX_SIZE_PIX;

            float weight_torque = BOX_MASS * g * (u / halfL) * STATIC_WEIGHT_SCALE;
            float d_omega       = (weight_torque / BOARD_INERTIA) * PHYSICS_DT;
            pplat.omega        += d_omega;
        }
    }

    // 4) BOARD SPRING/DAMPING 
    float torque = -BOARD_SPRING_K * pplat.angle - BOARD_DAMP * pplat.omega;
    pplat.omega += torque * PHYSICS_DT;
    pplat.angle += pplat.omega * PHYSICS_DT;

    float error = (pplat.angle - tilt_angle * DEGtoRAD) * 0.7f;
    pplat.angle -= error;

    // This will need to be changed to a torque
    // pplat.angle = tilt_angle * DEGtoRAD;

    // Clamp board angle to +/- 30 degrees
    const float max_angle_rad = 30.0f * DEGtoRAD;
    if (pplat.angle >  max_angle_rad) pplat.angle =  max_angle_rad;
    if (pplat.angle < -max_angle_rad) pplat.angle = -max_angle_rad;

    board_angle_deg = pplat.angle * DEGtoRAD;

    // 5) CLEANUP GRID CELLS THAT SLID OFF-SCREEN
    cleanup_grid(c, s);
}

char *get_random_greeting_phrase(void)
{
    uint16_t rand_greeting = rand()%3;

    switch (rand_greeting)
    {
        case 0:
            return GREETING_PHRASE_1;

        case 1:
            return GREETING_PHRASE_2;
            
        case 2:
            return GREETING_PHRASE_3;
    }
}

char *get_random_losing_phrase(void)
{
    uint16_t rand_losing = rand()%3;

    switch (rand_losing)
    {
        case 0:
            return LOSING_PHRASE_1;

        case 1:
            return LOSING_PHRASE_2;
            
        case 2:
            return LOSING_PHRASE_3;
    }
}

void display_main_screen(uint64_t time_left)
{
    static uint64_t display_drift = 0;
    static double title_display_offset_x = 0.0;
    static double title_display_offset_y = 0.0;
    static double greeting_display_offset_x = 0.0;
    static double greeting_display_offset_y = 0.0;
    static double controller_display_offset_x = 0.0;
    static double controller_display_offset_y = 0.0;
    static double controller_display2_offset_x = 0.0;
    static double controller_display2_offset_y = 0.0;
    static double time_display_offset_x = 0.0;
    static double time_display_offset_y = 0.0;
    static bool last_controller = false;
    static uint64_t last_time = 0;
    
    display_drift += 1;
    
    while(gpio_get(VSYNC)){};

    // Clear the old title
    setTextColor2(BLACK, BLACK);
    setTextSize(5);
    int print_width = strlen(TITLE) * (5 * 6);
    setCursor(SCREEN_CENTER_X - print_width/2 + title_display_offset_x, SCREEN_CENTER_Y - 50 + title_display_offset_y);
    writeString(TITLE);
    
    // Update the display drift
    title_display_offset_x = sin(display_drift * TITLE_DRIFT_RATE) * TITLE_DRIFT_MAX_X;
    title_display_offset_y = cos(display_drift * TITLE_DRIFT_RATE) * TITLE_DRIFT_MAX_Y;
    
    // Write the title
    setTextColor2(YELLOW, BLACK);
    setTextSize(5);
    print_width = strlen(TITLE) * (5 * 6);
    setCursor(SCREEN_CENTER_X - print_width/2 + title_display_offset_x, SCREEN_CENTER_Y - 50 + title_display_offset_y);
    writeString(TITLE);
    
    while(gpio_get(VSYNC)){};

    // Clear the old greeting
    setTextColor2(BLACK, BLACK);
    setTextSize(2);
    print_width = strlen(greeting_phrase) * (2 * 6);
    setCursor(SCREEN_CENTER_X - print_width/2 + greeting_display_offset_x, SCREEN_CENTER_Y + 10 + greeting_display_offset_y);
    writeString(greeting_phrase);
    
    // Update the greeting drift
    greeting_display_offset_x = sin(display_drift * GREETING_DRIFT_RATE) * GREETING_DRIFT_MAX_X;
    greeting_display_offset_y = cos(display_drift * GREETING_DRIFT_RATE) * GREETING_DRIFT_MAX_Y;
    
    // Write the greeting
    setTextColor2(DARK_ORANGE, BLACK);
    setTextSize(2);
    print_width = strlen(greeting_phrase) * (2 * 6);
    setCursor(SCREEN_CENTER_X - print_width/2 + greeting_display_offset_x, SCREEN_CENTER_Y + 10 + greeting_display_offset_y);
    writeString(greeting_phrase);
    
    while(gpio_get(VSYNC)){};

    // Clear the old controller prompt
    setTextColor2(BLACK, BLACK);

    if (last_controller == false)
    {
        setTextSize(2);
        print_width = strlen(CONTROLLER_PHRASE_1) * (2 * 6);
        setCursor(SCREEN_CENTER_X - print_width/2 + controller_display_offset_x, SCREEN_CENTER_Y + 40 + controller_display_offset_y);
        writeString(CONTROLLER_PHRASE_1);
    }
    else
    {
        setTextSize(2);
        print_width = strlen(CONTROLLER_PHRASE_2) * (2 * 6);
        setCursor(SCREEN_CENTER_X - print_width/2 + controller_display_offset_x, SCREEN_CENTER_Y + 40 + controller_display_offset_y);
        writeString(CONTROLLER_PHRASE_2);
        
        print_width = strlen(CONTROLLER_PHRASE_3) * (2 * 6);
        setCursor(SCREEN_CENTER_X - print_width/2 + controller_display2_offset_x, SCREEN_CENTER_Y + 70 + controller_display2_offset_y);
        writeString(CONTROLLER_PHRASE_3);
    }

    // Update the controller prompt drift
    controller_display_offset_x = sin(display_drift * CONTROLLER_DRIFT_RATE) * CONTROLLER_DRIFT_MAX_X;
    controller_display_offset_y = cos(display_drift * CONTROLLER_DRIFT_RATE) * CONTROLLER_DRIFT_MAX_Y;
    controller_display2_offset_x = sin(display_drift * CONTROLLER_DRIFT_RATE + 12345) * CONTROLLER_DRIFT_MAX_X;
    controller_display2_offset_y = cos(display_drift * CONTROLLER_DRIFT_RATE + 12345) * CONTROLLER_DRIFT_MAX_Y;
    last_controller = controller_is_connected();
    
    // Write the controller prompt
    if (last_controller == false)
    {
        setTextColor2(CONTROLLER_PHRASE_1_COLOR, BLACK);
        setTextSize(2);
        print_width = strlen(CONTROLLER_PHRASE_1) * (2 * 6);
        setCursor(SCREEN_CENTER_X - print_width/2 + controller_display_offset_x, SCREEN_CENTER_Y + 40 + controller_display_offset_y);
        writeString(CONTROLLER_PHRASE_1);
    }
    else
    {
        setTextColor2(CONTROLLER_PHRASE_2_COLOR, BLACK);
        setTextSize(2);
        print_width = strlen(CONTROLLER_PHRASE_2) * (2 * 6);
        setCursor(SCREEN_CENTER_X - print_width/2 + controller_display_offset_x, SCREEN_CENTER_Y + 40 + controller_display_offset_y);
        writeString(CONTROLLER_PHRASE_2);
        
        setTextColor2(CONTROLLER_PHRASE_3_COLOR, BLACK);
        print_width = strlen(CONTROLLER_PHRASE_3) * (2 * 6);
        setCursor(SCREEN_CENTER_X - print_width/2 + controller_display2_offset_x, SCREEN_CENTER_Y + 70 + controller_display2_offset_y);
        writeString(CONTROLLER_PHRASE_3);
    }
    
    while(gpio_get(VSYNC)){};

    if (last_time != 0)
    {
        setTextColor2(BLACK, BLACK);
        setTextSize(2);

        float seconds_to_start = (((float) last_time)/1000000);

        sprintf(str_buf, "Starting in: %.1fs", seconds_to_start);
        print_width = strlen(str_buf) * (2 * 6);
        setCursor(SCREEN_CENTER_X - print_width/2 + time_display_offset_x, SCREEN_CENTER_Y + 100 + time_display_offset_y);
        writeString(str_buf);
    }

    last_time = time_left;
    time_display_offset_x = sin(display_drift * TIME_DRIFT_RATE) * TIME_DRIFT_MAX_X;
    time_display_offset_y = cos(display_drift * TIME_DRIFT_RATE) * TIME_DRIFT_MAX_Y;

    if (last_time != 0)
    {
        setTextColor2(TIME_PHRASE_COLOR, BLACK);
        setTextSize(2);

        float seconds_to_start = (((float) last_time)/1000000);

        sprintf(str_buf, "Starting in: %.1fs", seconds_to_start);
        print_width = strlen(str_buf) * (2 * 6);
        setCursor(SCREEN_CENTER_X - print_width/2 + time_display_offset_x, SCREEN_CENTER_Y + 100 + time_display_offset_y);
        writeString(str_buf);
    }
}


void display_losing_screen(uint64_t time_left)
{
    static uint64_t display_drift = 0;
    static double title_display_offset_x = 0.0;
    static double title_display_offset_y = 0.0;
    static double losing_display_offset_x = 0.0;
    static double losing_display_offset_y = 0.0;
    static double time_display_offset_x = 0.0;
    static double time_display_offset_y = 0.0;
    static bool last_controller = false;
    static uint64_t last_time = 0;
    
    display_drift += 1;
    
    while(gpio_get(VSYNC)){};

    // Clear the old title
    setTextColor2(BLACK, BLACK);
    setTextSize(5);
    int print_width = strlen(YOU_LOSE) * (5 * 6);
    setCursor(SCREEN_CENTER_X - print_width/2 + title_display_offset_x, SCREEN_CENTER_Y - 30 + title_display_offset_y);
    writeString(YOU_LOSE);
    
    // Update the display drift
    title_display_offset_x = sin(display_drift * TITLE_DRIFT_RATE) * TITLE_DRIFT_MAX_X;
    title_display_offset_y = cos(display_drift * TITLE_DRIFT_RATE) * TITLE_DRIFT_MAX_Y;
    
    // Write the title
    setTextColor2(YOU_LOSE_COLOR, BLACK);
    setTextSize(5);
    print_width = strlen(YOU_LOSE) * (5 * 6);
    setCursor(SCREEN_CENTER_X - print_width/2 + title_display_offset_x, SCREEN_CENTER_Y - 30 + title_display_offset_y);
    writeString(YOU_LOSE);
    
    while(gpio_get(VSYNC)){};

    // Clear the old greeting
    setTextColor2(BLACK, BLACK);
    setTextSize(2);
    print_width = strlen(losing_phrase) * (2 * 6);
    setCursor(SCREEN_CENTER_X - print_width/2 + losing_display_offset_x, SCREEN_CENTER_Y + 30 + losing_display_offset_y);
    writeString(losing_phrase);
    
    // Update the greeting drift
    losing_display_offset_x = sin(display_drift * LOSING_DRIFT_RATE) * LOSING_DRIFT_MAX_X;
    losing_display_offset_y = cos(display_drift * LOSING_DRIFT_RATE) * LOSING_DRIFT_MAX_Y;
    
    // Write the losing message
    setTextColor2(RED, BLACK);
    setTextSize(2);
    print_width = strlen(losing_phrase) * (2 * 6);
    setCursor(SCREEN_CENTER_X - print_width/2 + losing_display_offset_x, SCREEN_CENTER_Y + 30 + losing_display_offset_y);
    writeString(losing_phrase);
    
    while(gpio_get(VSYNC)){};

    if (last_time != 0)
    {
        setTextColor2(BLACK, BLACK);
        setTextSize(2);

        float seconds_to_start = (((float) last_time)/1000000);

        sprintf(str_buf, "Restarting in: %.1fs", seconds_to_start);
        print_width = strlen(str_buf) * (2 * 6);
        setCursor(SCREEN_CENTER_X - print_width/2 + time_display_offset_x, SCREEN_CENTER_Y + 60 + time_display_offset_y);
        writeString(str_buf);
    }

    last_time = time_left;
    time_display_offset_x = sin(display_drift * TIME_DRIFT_RATE) * TIME_DRIFT_MAX_X;
    time_display_offset_y = cos(display_drift * TIME_DRIFT_RATE) * TIME_DRIFT_MAX_Y;

    if (last_time != 0)
    {
        setTextColor2(TIME_PHRASE_COLOR, BLACK);
        setTextSize(2);

        float seconds_to_start = (((float) last_time)/1000000);

        sprintf(str_buf, "Restarting in: %.1fs", seconds_to_start);
        print_width = strlen(str_buf) * (2 * 6);
        setCursor(SCREEN_CENTER_X - print_width/2 + time_display_offset_x, SCREEN_CENTER_Y + 60 + time_display_offset_y);
        writeString(str_buf);
    }
}

void create_display_block(display_block *block, float x, float y, float max_x_drift, float max_y_drift, float drift_rate, float spin_rate, uint16_t color, uint16_t size)
{
    block->x = x;
    block->y = y;
    block->max_x_drift = max_x_drift;
    block->max_y_drift = max_y_drift;
    block->drift_rate = drift_rate;
    block->spin_rate = spin_rate;
    block->color = color;
    block->size = size;
}

static display_block title_blocks[16];
static display_block game_blocks[4];
static display_block end_blocks[16];

void init_spinning_blocks(void)
{
    float x_off;
    float y_off;
    float spin_rate;
    float drift_rate;
    float max_x_drift;
    float max_y_drift;
    uint16_t size;

    for (float i = 0; i < 16; i = i + 1)
    {
        x_off = sin((1 + 2 * i) * (3.141592/16)) * 280;
        y_off = cos((1 + 2 * i) * (3.141592/16)) * 175;

        drift_rate = 0.03 + ((float) (rand()%1000)) * 0.00007;
        spin_rate = 0.03 + ((float) (rand()%1000)) * 0.00007;

        max_x_drift = 7 + (rand() % 5);
        max_y_drift = 7 + (rand() % 5);

        size = 17 + (rand() % 7);

        create_display_block(&title_blocks[(int) i], ((int) SCREEN_CENTER_X + x_off), ((int) SCREEN_CENTER_Y + y_off), max_x_drift, max_y_drift, drift_rate, spin_rate, rand_color(), size);
    }
    
    for (float i = 0; i < 8; i = i + 1)
    {
        x_off = sin((1 + 2 * i) * (3.141592/8)) * 280;
        y_off = cos((1 + 2 * i) * (3.141592/8)) * 175;

        drift_rate = 0.03 + ((float) (rand()%1000)) * 0.00007;
        spin_rate = 0.03 + ((float) (rand()%1000)) * 0.00007;

        max_x_drift = 7 + (rand() % 5);
        max_y_drift = 7 + (rand() % 5);

        size = 17 + (rand() % 7);

        create_display_block(&end_blocks[(int) i], ((int) SCREEN_CENTER_X + x_off), ((int) SCREEN_CENTER_Y + y_off), max_x_drift, max_y_drift, drift_rate, spin_rate, rand_color(), size);
    }
}

void display_spinning_blocks(void)
{
    static uint64_t accum = 123456;

    if (title_screen)
    {
        while(gpio_get(VSYNC)){};

        // Clear the title screen blocks
        for (int i = 0; i < 16; i++)
        {
            display_block *block = &title_blocks[i];

            float x_off = sin(accum * block->drift_rate) * block->max_x_drift;
            float y_off = cos(accum * block->drift_rate) * block->max_y_drift;

            float c = cos(accum * block->spin_rate);
            float s = sin(accum * block->spin_rate);

            drawBoxRotated(block->x + x_off, block->y + y_off, c, s, block->size, BLACK);
        }
        
        accum += 1; 

        // Display the title screen blocks
        for (int i = 0; i < 16; i++)
        {
            display_block *block = &title_blocks[i];

            float x_off = sin(accum * block->drift_rate) * block->max_x_drift;
            float y_off = cos(accum * block->drift_rate) * block->max_y_drift;

            float c = cos(accum * block->spin_rate);
            float s = sin(accum * block->spin_rate);

            drawBoxRotated(block->x + x_off, block->y + y_off, c, s, block->size, block->color);
        }
    }
    else if (end_screen)
    {
        while(gpio_get(VSYNC)){};

        // Clear the end screen blocks
        for (int i = 0; i < 16; i++)
        {
            display_block *block = &end_blocks[i];

            float x_off = sin(accum * block->drift_rate) * block->max_x_drift;
            float y_off = cos(accum * block->drift_rate) * block->max_y_drift;

            float c = cos(accum * block->spin_rate);
            float s = sin(accum * block->spin_rate);

            drawBoxRotated(block->x + x_off, block->y + y_off, c, s, block->size, BLACK);
        }
        
        accum += 1; 

        // Display the end screen blocks
        for (int i = 0; i < 16; i++)
        {
            display_block *block = &end_blocks[i];

            float x_off = sin(accum * block->drift_rate) * block->max_x_drift;
            float y_off = cos(accum * block->drift_rate) * block->max_y_drift;

            float c = cos(accum * block->spin_rate);
            float s = sin(accum * block->spin_rate);

            drawBoxRotated(block->x + x_off, block->y + y_off, c, s, block->size, block->color);
        }
    }
    else if (game_screen)
    {
        return;
        while(gpio_get(VSYNC)){};

        // Clear the game screen blocks
        for (int i = 0; i < 4; i++)
        {
            display_block *block = &game_blocks[i];

            float x_off = sin(accum * block->drift_rate) * block->max_x_drift;
            float y_off = cos(accum * block->drift_rate) * block->max_y_drift;

            float c = cos(accum * block->spin_rate);
            float s = sin(accum * block->spin_rate);

            drawBoxRotated(block->x + x_off, block->y + y_off, c, s, block->size, BLACK);
        }
        
        accum += 1; 

        // Display the game screen blocks
        for (int i = 0; i < 4; i++)
        {
            display_block *block = &game_blocks[i];

            float x_off = sin(accum * block->drift_rate) * block->max_x_drift;
            float y_off = cos(accum * block->drift_rate) * block->max_y_drift;

            float c = cos(accum * block->spin_rate);
            float s = sin(accum * block->spin_rate);

            drawBoxRotated(block->x + x_off, block->y + y_off, c, s, block->size, block->color);
        }
    }
}

// ---------------------------------------------------------
// 6. PROTOTHREAD: VGA DRAW
// ---------------------------------------------------------
static PT_THREAD (protothread_vga(struct pt *pt))
{
    PT_BEGIN(pt);

    init_spinning_blocks();
    set_song(SONG_MENU);

    static uint64_t begin_time;
    static uint64_t spare_time;

    while (true)
    {
        fillRect(0, 0, 640, 480, BLACK);

        greeting_phrase = get_random_greeting_phrase();
        losing_phrase = get_random_losing_phrase();
        start_delay = START_DELAY_MAX;

        title_screen = true;
        end_screen = false;
        game_screen = false;

        while (true)
        {
            while (true)
            {
                begin_time = time_us_64();

                display_main_screen(0);
                display_spinning_blocks();

                if (controller_is_connected() && abs((int) tilt_angle) > 10)
                {
                    break;
                }

                spare_time = 16000 - (time_us_64() - begin_time);

                PT_YIELD_usec(spare_time);
            }
            
            uint64_t pickup_time_us = time_us_64();

            while (controller_is_connected())
            {   
                uint64_t now_us = time_us_64();
                begin_time = time_us_64();

                if (now_us - pickup_time_us > START_DELAY_MAX_US || !controller_is_connected())
                {
                    break;
                }

                display_main_screen(START_DELAY_MAX_US - (now_us - pickup_time_us));
                display_spinning_blocks();
                
                spare_time = 16000 - (time_us_64() - begin_time);

                PT_YIELD_usec(spare_time);
            }

            if (controller_is_connected())
            {
                break;
            }
        }

        title_screen = false;
        game_screen = true;
        set_song(SONG_GAME);

        fillRect(0, 0, 640, 480, BLACK);

        physics_init();
        
        c = cosf(pplat.angle);
        s = sinf(pplat.angle);

        setTextSize(1);
        setTextColor2(WHITE, BLACK);
            
        float halfL = 0.5f * (float)BOARD_WIDTH;
        float halfH = 0.5f * (float)BOARD_HEIGHT;
        
        while (true)
        {
            begin_time = time_us_64();
            while(gpio_get(VSYNC)){};

            // Hide board
            hideBoardPaddle(c, s);
            
            for (int col = 0; col < GRID_COLS; col++) {
                for (int row = 0; row < GRID_ROWS; row++) {
                    if (!grid[col][row].occupied) continue;
                    float x, y;
                    grid_cell_world_center(col, row, &x, &y, c, s, halfL, halfH);
                    drawBoxRotated(x, y, c, s, BOX_SIZE_PIX-1, BLACK);
                }
            }

            // Hide falling boxes (red, axis-aligned)
            for (int i = 0; i < MAX_FALLING; i++) {
                if (!falling[i].active) continue;
                drawBoxRotated(falling[i].x, falling[i].y, 1.0f, 0.0f, BOX_SIZE_PIX-1, BLACK);
            }

            c = cosf(pplat.angle);
            s = sinf(pplat.angle);

            // Step physics
            physics_step();

            for (int col = 0; col < GRID_COLS; col++) {
                for (int row = 0; row < GRID_ROWS; row++) {
                    if (!grid[col][row].occupied) continue;
                    float x, y;
                    grid_cell_world_center(col, row, &x, &y, c, s, halfL, halfH);
                    drawBoxRotated(x, y, c, s, BOX_SIZE_PIX-1, grid[col][row].color);
                }
            }

            // Draw falling boxes (red, axis-aligned)
            for (int i = 0; i < MAX_FALLING; i++) {
                if (!falling[i].active) continue;
                drawBoxRotated(falling[i].x, falling[i].y, 1.0f, 0.0f, BOX_SIZE_PIX-1, falling[i].color);
            }
            
            // Draw board
            drawBoardPaddle(c, s);

            // ----------------------------
            // Health bar + Timer
            // ----------------------------

            // Time since game started
            float elapsed_s;
            if (!game_over) {
                uint64_t now_us = time_us_64();
                if (game_start_us == 0) game_start_us = now_us;
                elapsed_s = (now_us - game_start_us) / 1000000.0f;
            } else {
                elapsed_s = final_time_s;
            }

            setCursor(SCREEN_LEFT_X + 10, SCREEN_TOP_Y + 10);
            sprintf(str_buf, "Time: %.1fs", elapsed_s);
            writeString(str_buf);

            // Health bar on the far right
            int bar_x = SCREEN_RIGHT_X - 20;
            int bar_y = SCREEN_TOP_Y + 40;
            int bar_w = 10;
            int bar_h = 200;

            // Bar border
            drawRect(bar_x, bar_y, bar_w, bar_h, WHITE);

            // Filled portion (from bottom up)
            if (health_points > 0) {
                float frac = (float)health_points / (float)MAX_HEALTH_POINTS;
                int filled = (int)(frac * (bar_h - 2));   // leave border

                if (filled < 0) filled = 0;
                if (filled > bar_h - 2) filled = bar_h - 2;

                int fy = bar_y + (bar_h - 1 - filled);
                fillRect(bar_x + 1, fy, bar_w - 2, filled, GREEN);
            }

            // HP label
            setCursor(SCREEN_RIGHT_X - 70, SCREEN_TOP_Y + 20);
            sprintf(str_buf, "HP: %2d", health_points);
            writeString(str_buf);
            
            // Decorative blocks for game screen
            display_spinning_blocks();

            // Game over message
            if (game_over)
            {
                game_screen = false;
                end_screen = true;
                set_song(SONG_MENU);

                fillRect(0, 0, 640, 480, BLACK);

                uint64_t game_over_time_us = time_us_64();
                uint64_t now_time_us = time_us_64();

                while (now_time_us <= game_over_time_us + GAME_OVER_TIME_US)
                {
                    display_losing_screen(GAME_OVER_TIME_US - (now_time_us - game_over_time_us));
                    display_spinning_blocks();
                    
                    PT_YIELD_usec(16000);
                    now_time_us = time_us_64();
                }
                
                game_over = false;
                break;
            }

            printf("Spare time: %d\n", spare_time);

            PT_YIELD_usec(spare_time);
        }
    }

    PT_END(pt);
}

void set_tilt_angle(float angle)
{
    tilt_angle = angle;
}

// ---------------------------------------------------------
// 7. CORE 1 ENTRY
// ---------------------------------------------------------

void core1_entry() {
    pt_add_thread(protothread_vga);
    pt_schedule_start;
}

// ---------------------------------------------------------
// 8. MAIN
// ---------------------------------------------------------

int main() {
    set_sys_clock_khz(150000, true);

    stdio_init_all();
    srand((unsigned int) time_us_32());

    initVGA();

    music_init();

    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    bt_client_main();

    // Nothing on core 0 (all work is on core 1)
    while (1) {
        tight_loop_contents();
    }

    return 0;
}


