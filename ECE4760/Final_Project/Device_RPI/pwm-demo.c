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

// ---------------------------------------------------------
// BOXES GAME parameters
// ---------------------------------------------------------
// Screen boundaries
#define SCREEN_TOP_Y 5
#define SCREEN_BOTTOM_Y 475
#define SCREEN_LEFT_X 5
#define SCREEN_RIGHT_X 635
// ---- Rotating board (paddle) params ----
static float board_angle_deg = 15.0f;     // <-- control this variable to rotate the board
#define BOARD_WIDTH   300
#define BOARD_HEIGHT  12
#define BOARD_Y       (SCREEN_BOTTOM_Y - 50)   // vertical position of the board center
#define BOARD_X       ((SCREEN_LEFT_X + SCREEN_RIGHT_X)/2)  // centered horizontally
#define MAX_BOXES 1
// the color of the box
char color = WHITE;
// Simulation parameters
static fix15 box_gravity = float2fix15(0.75f);
#define BOX_START_X 320
#define BOX_START_Y 25
#define BOX_START_DX_MAX 1



// ---------------------------------------------------------
// BOXES GAME parameters END
// ---------------------------------------------------------


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

fix15 accel_angle_x = int2fix15(0);
fix15 gyro_angle_delta_x = int2fix15(0);
fix15 complementary_angle_x = int2fix15(0);
fix15 accel_angle_y = int2fix15(0);
fix15 gyro_angle_delta_y = int2fix15(0);
fix15 complementary_angle_y = int2fix15(0);


// -------------------------------------------------------------------------
// Rotated rectangle functions
// -------------------------------------------------------------------------

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


// --------------------------------------------------------------------------
// Rotated rectangle functions END
// --------------------------------------------------------------------------




// PWM interrupt service routine
void on_pwm_wrap() {
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_OUT));
    // Update duty cycle
    if (control!=old_control) {
        old_control = control ;
        pwm_set_chan_level(slice_num, PWM_CHAN_A, control);
    }
    
    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    mpu6050_read_raw(acceleration, gyro);

    // SMALL ANGLE APPROXIMATION
    accel_angle_x = multfix15(divfix(acceleration[1], acceleration[2]), oneeightyoverpi) ;
    accel_angle_y = multfix15(divfix(acceleration[0], acceleration[2]), oneeightyoverpi) ;
    // NO SMALL ANGLE APPROXIMATION
    // accel_angle = multfix15(float2fix15(atan2(-filtered_ax, filtered_ay) + PI), oneeightyoverpi);

    // Gyro angle delta (measurement times timestep) (15.16 fixed point)
    gyro_angle_delta_x = multfix15(gyro[0], zeropt001) ;
    gyro_angle_delta_y = multfix15(gyro[1], zeropt001) ;

    // Complementary angle (degrees - 15.16 fixed point)
    complementary_angle_x = multfix15(complementary_angle_x - gyro_angle_delta_x, zeropt999) - multfix15(accel_angle_x, zeropt001);
    complementary_angle_y = multfix15(complementary_angle_y + gyro_angle_delta_y, zeropt999) - multfix15(accel_angle_y, zeropt001);

    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);
}

// User input thread
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static int test_in ;
    static char mode;
    static float value;

    while(1) {
        sprintf(pt_serial_out_buffer, "change value (p, i, d, a):");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%c", &mode) ;
        
        sprintf(pt_serial_out_buffer, "new value:");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        sscanf(pt_serial_in_buffer,"%f", &value) ;

        //if (test_in > 50) continue ;
        //else if (test_in < -50) continue ;
        //else angle = int2fix15(test_in) ;
    }
    PT_END(pt) ;
}

static PT_THREAD (protothread_button(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

    while (true)
    {
        {
		    PT_YIELD_usec(1000);
        }
    }

    // Indicate end of thread
    PT_END(pt);
}

// Thread that draws to VGA display
static PT_THREAD (protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

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

    // Draw bottom plot
    drawHLine(75, 430, 5, CYAN) ;
    drawHLine(75, 355, 5, CYAN) ;
    drawHLine(75, 280, 5, CYAN) ;
    drawVLine(80, 280, 150, CYAN) ;
    sprintf(screentext, "5000") ;
    setCursor(50, 280) ;
    writeString(screentext) ;
    sprintf(screentext, "0") ;
    setCursor(50, 425) ;
    writeString(screentext) ;

    // Draw top plot
    drawHLine(75, 230, 5, CYAN) ;
    drawHLine(75, 155, 5, CYAN) ;
    drawHLine(75, 80, 5, CYAN) ;
    drawVLine(80, 80, 150, CYAN) ;
    sprintf(screentext, "90") ;
    setCursor(45, 75) ;
    writeString(screentext) ;
    sprintf(screentext, "0") ;
    setCursor(50, 150) ;
    writeString(screentext) ;
    sprintf(screentext, "-90") ;
    setCursor(45, 225) ;
    writeString(screentext) ;
    

    while (true) {
        fillRect(0, 0, 640, 480, BLACK);
        drawBoard();
        drawBoardPaddle();

        // Wait on semaphore
        PT_SEM_WAIT(pt, &vga_semaphore);
        // Increment drawspeed controller
        throttle += 1 ;
        // If the controller has exceeded a threshold, draw
        if (throttle >= threshold) { 
            // Zero drawspeed controller
            throttle = 0 ;

            {
                // fillRect(0, 0, 500, 75, BLACK);
                
                sprintf(screentext, "Angle around X axis: %.02f    ", fix2float15(complementary_angle_x));
                setCursor(30, 15);
                writeString(screentext);
                sprintf(screentext, "Angle around Y axis: %.02f    ", fix2float15(complementary_angle_y));
                setCursor(30, 30);
                writeString(screentext);
            }

            // Erase a column
            drawVLine(xcoord, 75, 480, BLACK) ;

            // Draw bottom plot (multiply by 120 to scale from +/-2 to +/-250)
            // drawPixel(xcoord, 430 - (int)(NewRange*((float)((((float)(control-2500))/10)-OldMin)/OldRange)), WHITE) ;

            // Draw top plot
            // drawPixel(xcoord, 230 - (int)(NewRange*((float)(((fix2float15(angle)-90.0)*-2.5f)-OldMin)/OldRange)), GREEN) ;
            drawPixel(xcoord, 230 - (int)(NewRange*((float)(((fix2float15(complementary_angle_x))*-2.5f)-OldMin)/OldRange)), RED) ;
            drawPixel(xcoord, 230 - (int)(NewRange*((float)(((fix2float15(complementary_angle_y))*-2.5f)-OldMin)/OldRange)), BLUE) ;

            // Update horizontal cursor
            if (xcoord < 609) {
                xcoord += 1 ;
            }
            else {
                xcoord = 81 ;
            }
           
        }
    }
    // Indicate end of thread
    PT_END(pt);
}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_vga) ;
    pt_add_thread(protothread_button) ;
    pt_schedule_start ;
}

int main() {

    // Overclock
    set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();

    printf("Starting\n");

    // Initialize VGA
    initVGA() ;

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;

    // Pullup resistors on breakout board, don't need to turn on internals
    // gpio_pull_up(SDA_PIN) ;
    // gpio_pull_up(SCL_PIN) ;

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO PWM_OUT that it is allocated to the PWM
    gpio_set_function(5, GPIO_FUNC_PWM);
    // gpio_set_function(4, GPIO_FUNC_PWM);
    gpio_set_function(PWM_OUT, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO PWM_OUT (it's slice 2)
    slice_num = pwm_gpio_to_slice_num(PWM_OUT);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 3125);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    pt_add_thread(protothread_serial) ;
    pt_schedule_start ;

}
