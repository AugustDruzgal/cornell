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

#include <stdio.h>
#include <math.h>

// BTstack
#include "btstack.h"

// High-level libraries
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "pico/sync.h"

// Include custom libraries
#include "VGA/vga16_graphics_v2.h"
#include "MPU6050/mpu6050.h"

// Hardware API's
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "hardware/sync.h"
#include "hardware/clocks.h"

// GAP and GATT
#include "GAP_Advertisement/gap_config.h"
#include "GATT_Service/service_implementation.h"

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

// Period with which we'll enter the BTstack timer callback
#define HEARTBEAT_PERIOD_MS 250

// BTstack objects
static btstack_timer_source_t heartbeat;
static btstack_packet_callback_registration_t hci_event_callback_registration;

// Some data that we will communicate over Bluetooth
static int characteristic_a_val = 0 ;

// We send data as formatted strings (just like a serial console)
static char characteristic_a_tx[255] ;

static void update_mag_value(void);

// Heartbeat protothread
static PT_THREAD (protothread_heartbeat(struct pt *pt))
{
    PT_BEGIN(pt) ;

    // Buffer for writing counter to VGA
    static char countval[20] ;

    while(1) {

        // Increment the counter
        characteristic_a_val += 1 ;

        // Update characteristic (sends to client if notifications enabled)
        // set_characteristic_a_value(characteristic_a_val) ;

        // Yield
        PT_YIELD_usec(500000) ;
    }

    PT_END(pt) ;
}

// Protothread that handles received Bluetooth data
static PT_THREAD (protothread_ble(struct pt *pt))
{
    PT_BEGIN(pt);

    while(1) {
        // Wait for a bluetooth event (signaled by bluetooth write callback)
        PT_SEM_SDK_WAIT(pt, &BLUETOOTH_READY) ;
    }

  PT_END(pt);
}

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

    // printf("Value: %f", fix2float15(complementary_angle_x));
    static int counter = 0;

    counter++;

    if (counter%200==0)
    {
        update_mag_value();
    }

    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);
}

static void update_mag_value(void)
{
    set_characteristic_a_value(fix2float15(complementary_angle_x));
}

// Serial input thread
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);

      while(1)
      {
        // print prompt
        sprintf(pt_serial_out_buffer, "Input a string: ");
        // spawn a thread to do the non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
      } // END WHILE(1)

  PT_END(pt);
} // timer thread

/*
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
*/

/*
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
*/

// Entry point for core 1
void core1_entry() {
    // pt_add_thread(protothread_vga) ;
// pt_add_thread(protothread_button) ;
    pt_schedule_start ;
}

int main() {

    // Overclock
    set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();

    printf("Starting\n");

    // Initialize VGA
    // initVGA() ;

    // initialize CYW43 driver architecture (will enable BT if/because CYW43_ENABLE_BLUETOOTH == 1)
    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }

    // Initialize L2CAP and security manager
    l2cap_init();
    sm_init();

    // Initialize ATT server, no general read/write callbacks
    // because we'll set one up for each service
    att_server_init(profile_data, NULL, NULL);   

    // Instantiate our custom service handler
    custom_service_server_init() ;

    // inform about BTstack state
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // register for ATT event
    att_server_register_packet_handler(packet_handler);

    // turn on bluetooth!
    hci_power_control(HCI_POWER_ON);

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
    // multicore_reset_core1();
    // multicore_launch_core1(core1_entry);

    pt_add_thread(protothread_ble);
    pt_add_thread(protothread_serial);
    // pt_add_thread(protothread_heartbeat) ;
    pt_sched_method = SCHED_ROUND_ROBIN ;
    pt_schedule_start ;
}
