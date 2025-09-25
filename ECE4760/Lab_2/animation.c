
/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS
	- GPIO 16 ---> VGA Hsync
	- GPIO 17 ---> VGA Vsync
	- GPIO 18 ---> VGA Green lo-bit --> 470 ohm resistor --> VGA_Green
	- GPIO 19 ---> VGA Green hi_bit --> 330 ohm resistor --> VGA_Green
	- GPIO 20 ---> 330 ohm resistor ---> VGA-Blue
	- GPIO 21 ---> 330 ohm resistor ---> VGA-Red
	- RP2040 GND ---> VGA-GND
*
* RESOURCES USED
	- PIO state machines 0, 1, and 2 on PIO instance 0
	- DMA channels (2, by claim mechanism)
	- 153.6 kBytes of RAM (for pixel color data)
*
*/

// Include the VGA grahics library
#include "vga16_graphics_v2.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"

// === the fixed point macros ========================================
typedef signed int fix15;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))

#define SCREEN_TOP_Y 40
#define SCREEN_BOTTOM_Y 440
#define SCREEN_LEFT_X 40
#define SCREEN_RIGHT_X 620

// Wall detection
#define hitBottom(b) (b>int2fix15(SCREEN_BOTTOM_Y))
#define hitTop(b) (b<int2fix15(SCREEN_TOP_Y))
#define hitLeft(a) (a<int2fix15(SCREEN_LEFT_X))
#define hitRight(a) (a>int2fix15(SCREEN_RIGHT_X))

// uS per frame
#define FRAME_RATE 33000

// the color of the ball
char color = WHITE;

typedef struct _ball {
	fix15 x;
	fix15 y;
	fix15 vx;
	fix15 vy;
	int last_peg;
} ball_t;

typedef struct _peg {
	fix15 x;
	fix15 y;
} peg_t;

#define MAX_BALLS 256
#define BALL_RADIUS 4
#define BALL_BOUNCINESS 0.5f
#define BALL_GRAVITY 0.75f
#define BALL_START_DX_MAX 1.0f
#define BALL_START_X 320
#define BALL_START_Y 120
#define BALLS_COUNT 10
static ball_t balls[MAX_BALLS];
static int balls_len = 0;

#define GALTON_HEIGHT 16
#define GALTON_H_SEPARATION 38
#define GALTON_V_SEPARATION 19
#define GALTON_TOP_Y 150

#define MAX_PEGS 256
#define PEG_RADIUS 6
static peg_t pegs[MAX_PEGS];
static int pegs_len = 0;

// Number of samples per period in sine table
#define sine_table_size 256

// Sine table
int raw_sin[sine_table_size];

// Table of values to be sent to DAC
unsigned short DAC_data[sine_table_size];

// Pointer to the address of the DAC data table
unsigned short * _address_pointer = &DAC_data[0];

static int data_chan;
static int ctrl_chan;

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000

//SPI configurations
#define PIN_SCK  2
#define PIN_MISO 3
#define PIN_MOSI 4
#define PIN_CS   5
#define SPI_PORT spi0

// Number of DMA transfers per event
const uint32_t transfer_count = sine_table_size;

void resetBall(ball_t *ball)
{
	ball->last_peg = -1;
	ball->x = int2fix15(BALL_START_X);
	ball->y = int2fix15(BALL_START_Y);
	ball->vy = 0;

	int r = rand() % 1000;
	r -= 500;
	
	ball->vx = multfix15(divfix(int2fix15(r), int2fix15(500)), float2fix15(BALL_START_DX_MAX));
}

// Create a ball
void spawnball(void)
{
	ball_t *ball = &balls[balls_len++];
	resetBall(ball);
}

void play_hit_sound(void)
{
	dma_channel_start(ctrl_chan);
}

void updateBall(ball_t *ball)
{
	fix15 dx; 
	fix15 dy;
	fix15 dist; 
	fix15 normal_x;
	fix15 normal_y;
	fix15 temp;
	peg_t *peg;

	for (int i = 0; i < pegs_len; i++)
	{
		peg = &pegs[i];
		dx = ball->x - peg->x;
		dy = ball->y - peg->y;

		// check if the ball is overlapping the peg
		if (fix2float15(absfix15(dx)) >= BALL_RADIUS + PEG_RADIUS || fix2float15(absfix15(dy)) >= BALL_RADIUS + PEG_RADIUS)
		{
			continue;
		}

		dist = float2fix15((float) sqrt((double) fix2float15(multfix15(dx, dx) + multfix15(dy, dy))));

		normal_x = divfix(dx, dist);
		normal_y = divfix(dy, dist);

		temp = multfix15(int2fix15(-2), (multfix15(normal_x, ball->vx) + multfix15(normal_y, ball->vy)));

		if (temp > 0)
		{
			ball->x = peg->x + multfix15(normal_x, dist + int2fix15(1));
			ball->y = peg->y + multfix15(normal_y, dist + int2fix15(1));

			ball->vx = ball->vx + multfix15(normal_x, temp);
			ball->vy = ball->vy + multfix15(normal_y, temp);

			if (i != ball->last_peg)
			{
				ball->last_peg = i;

				play_hit_sound();

				ball->vx = multfix15(ball->vx, float2fix15(BALL_BOUNCINESS));
				ball->vy = multfix15(ball->vy, float2fix15(BALL_BOUNCINESS));
			}
		}
	}

	if (hitBottom(ball->y))
	{
		resetBall(ball);
	}

	if (hitTop(ball->y)) 
	{
		ball->vy = (-ball->vy);
		ball->y  = (ball->y + int2fix15(5));
	}

	if (hitRight(ball->x)) 
	{
		ball->vx = (-ball->vx);
		ball->x  = (ball->x - int2fix15(5));
	}

	if (hitLeft(ball->x)) 
	{
		ball->vx = (-ball->vx);
		ball->x  = (ball->x + int2fix15(5));
	} 

	ball->vy += float2fix15(BALL_GRAVITY);

	ball->x += ball->vx;
	ball->y += ball->vy;
}

void spawnPeg(int x, int y)
{
	peg_t *peg = &pegs[pegs_len];
	peg->x = int2fix15(x);
	peg->y = int2fix15(y);
	pegs_len += 1;
}

void createGalton()
{
	int startx = 0;

	for (int i = 0; i < GALTON_HEIGHT; i++)
	{
		startx = 320 - (int) (((float) (i))/2.0 * GALTON_H_SEPARATION);

		for (int j = 0; j <= i; j++)
		{
			spawnPeg(startx + j * GALTON_H_SEPARATION, GALTON_TOP_Y + i * GALTON_V_SEPARATION);
		}
	}
}

// Draw the boundaries
void drawArena() 
{
	drawVLine(SCREEN_RIGHT_X, SCREEN_TOP_Y, SCREEN_BOTTOM_Y - SCREEN_TOP_Y, WHITE);
	drawVLine(SCREEN_LEFT_X, SCREEN_TOP_Y, SCREEN_BOTTOM_Y - SCREEN_TOP_Y, WHITE);
	drawHLine(SCREEN_LEFT_X, SCREEN_BOTTOM_Y, SCREEN_RIGHT_X - SCREEN_LEFT_X, WHITE);
	drawHLine(SCREEN_LEFT_X, SCREEN_TOP_Y, SCREEN_RIGHT_X - SCREEN_LEFT_X, WHITE);
}

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
	PT_BEGIN(pt);
	// stores user input
	static int user_input;

	// wait for 0.1 sec
	PT_YIELD_usec(1000000);
	// announce the threader version
	sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
	// non-blocking write
	serial_write;

	while(1) 
	{
		// print prompt
		sprintf(pt_serial_out_buffer, "input a number in the range 1-15: ");
		// non-blocking write
		serial_write;
		// spawn a thread to do the non-blocking serial read
		serial_read;
		// convert input string to number
		sscanf(pt_serial_in_buffer,"%d", &user_input);

		// update ball color
		if ((user_input > 0) && (user_input < 16)) 
		{
			color = (char)user_input;	
		}
	} // END WHILE(1)

	PT_END(pt);
} // timer thread

// Animation on core 0
static PT_THREAD (protothread_anim(struct pt *pt))
{
	// Mark beginning of thread
	PT_BEGIN(pt);

	// Variables for maintaining frame rate
	static int begin_time;
	static int spare_time;

	for (int i = 0; i < BALLS_COUNT; i++)
	{
		spawnball();
	}
	// Spawn a ball

	createGalton();

	while(1) 
	{
		// TODO: handle multiple balls and pegs
		// Measure time at start of thread
		begin_time = time_us_32();      

		for (int i = 0; i < balls_len; i++)
		{
			// erase ball
			drawCircle(fix2int15(balls[i].x), fix2int15(balls[i].y), 4, BLACK);
			// update ball's position and velocity, check for peg hits
			updateBall(&balls[i]);
			// draw the ball at its new position
			drawCircle(fix2int15(balls[i].x), fix2int15(balls[i].y), 4, color);
		}

		for (int i = 0; i < pegs_len; i++)
		{
			drawCircle(fix2int15(pegs[i].x), fix2int15(pegs[i].y), 6, color);
		}

		// draw the boundaries
		drawArena();

		// delay in accordance with frame rate
		spare_time = FRAME_RATE - (time_us_32() - begin_time);
		// yield for necessary amount of time
		PT_YIELD_usec(spare_time);
		// NEVER exit while
	} // END WHILE(1)

	PT_END(pt);
} // animation thread

// Animation on core 1
static PT_THREAD (protothread_anim1(struct pt *pt))
{
	// Mark beginning of thread
	PT_BEGIN(pt);

	PT_END(pt);
} // animation thread

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main()
{
	// Add animation thread
	pt_add_thread(protothread_anim1);

	// Start the scheduler
	pt_schedule_start;
}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main()
{
	// Initidalize stdio
    stdio_init_all();

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000);

    // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports, acts like framed SPI with this CS mapping
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Build sine table and DAC data table
    int i;
    for (i=0; i<(sine_table_size); i++)
	{
        raw_sin[i] = (int)(2047 * sin((float)i*6.283/(float)sine_table_size) + 2047); //12 bit
        DAC_data[i] = DAC_config_chan_A | (raw_sin[i] & 0x0fff);
    }

    // Select DMA channels
    data_chan = dma_claim_unused_channel(true);;
    ctrl_chan = dma_claim_unused_channel(true);;

    // Setup the control channel
    dma_channel_config c = dma_channel_get_default_config(ctrl_chan);   // default configs
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);             // 32-bit txfers
    channel_config_set_read_increment(&c, false);                       // no read incrementing
    channel_config_set_write_increment(&c, false);                      // no write incrementing
    channel_config_set_chain_to(&c, data_chan);                         // chain to data channel

    dma_channel_configure(
        ctrl_chan,                          // Channel to be configured
        &c,                                 // The configuration we just created
        &dma_hw->ch[data_chan].read_addr,   // Write address (data channel read address)
        &_address_pointer,                   // Read address (POINTER TO AN ADDRESS)
        1,                                  // Number of transfers
        false                               // Don't start immediately
    );

    // Setup the data channel
    dma_channel_config c2 = dma_channel_get_default_config(data_chan);  // Default configs
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);            // 16-bit txfers
    channel_config_set_read_increment(&c2, true);                       // yes read incrementing
    channel_config_set_write_increment(&c2, false);                     // no write incrementing
    // (X/Y)*sys_clk, where X is the first 16 bytes and Y is the second
    // sys_clk is 125 MHz unless changed in code. Configured to ~44 kHz
    dma_timer_set_fraction(0, 0x0017, 0xffff);
    // 0x3b means timer0 (see SDK manual)
    channel_config_set_dreq(&c2, 0x3b);                                 // DREQ paced by timer 0
    // chain to the controller DMA channel
    // channel_config_set_chain_to(&c2, ctrl_chan);                        // Chain to control channel

    dma_channel_configure(
        data_chan,                  // Channel to be configured
        &c2,                        // The configuration we just created
        &spi_get_hw(SPI_PORT)->dr,  // write address (SPI data register)
        DAC_data,                   // The initial read address
        sine_table_size,            // Number of transfers
        false                       // Don't start immediately.
    );

    // start the control channel
    dma_start_channel_mask(1u << ctrl_chan);

	// VGA stuff
	set_sys_clock_khz(150000, true);

	// initialize VGA
	initVGA();

	// start core 1 
	// multicore_reset_core1();
	// multicore_launch_core1(&core1_main);

	// add threads
	pt_add_thread(protothread_serial);
	pt_add_thread(protothread_anim);

	// start scheduler
	pt_schedule_start;
} 
