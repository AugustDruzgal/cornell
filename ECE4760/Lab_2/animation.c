
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
#include "hardware/adc.h"
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

// Screen boundaries
#define SCREEN_TOP_Y 5
#define SCREEN_BOTTOM_Y 475
#define SCREEN_LEFT_X 5
#define SCREEN_RIGHT_X 635

// Wall detection
#define hitBottom(b) (b>int2fix15(GALTON_BOTTOM_Y))
#define hitTop(b) (b<int2fix15(SCREEN_TOP_Y))
#define hitLeft(a) (a<int2fix15(SCREEN_LEFT_X))
#define hitRight(a) (a>int2fix15(SCREEN_RIGHT_X))

// uS per frame
#define FRAME_RATE 33000
#define VSYNC 17

// the color of the ball
char color = WHITE;

// Peg definition
typedef struct _peg {
	fix15 x;
	fix15 y;
} peg_t;

// Ball definition
typedef struct _ball {
	fix15 x;
	fix15 y;
	fix15 vx;
	fix15 vy;
	peg_t *last_peg;
} ball_t;

// Button mode definitions
typedef enum _button_mode {
	BUTTON_MODE_NUM_BALLS,
	BUTTON_MODE_BOUNCINESS,
	BUTTON_MODE_GRAVITY,
	BUTTON_MODE_NOTHING
} button_mode;

// Button state machine states
typedef enum _button_state {
    BUTTON_NOT_PRESSED,
    BUTTON_MAYBE_PRESSED,
    BUTTON_PRESSED,
    BUTTON_MAYBE_NOT_PRESSED
} button_state;

// Ball configuration
#define MIN_BALLS 1
#define MAX_BALLS 4096
#define BALL_RADIUS_INT 1
#define BALL_RADIUS_FIX int2fix15(BALL_RADIUS_INT)
#define MAX_GRAVITY 2.0f
#define MIN_GRAVITY 0.1f
#define MAX_BOUNCINESS 0.9f
#define MIN_BOUNCINESS 0.1f
#define BALL_START_DX_MAX 1
#define BALL_START_X 320
#define BALL_START_Y 25

// Ball array
static ball_t balls[MAX_BALLS];
static int balls_len = 0;

// Galton board configuration
#define GALTON_HEIGHT 16
#define GALTON_MAX_HEIGHT 16
#define GALTON_H_SEPARATION 38
#define GALTON_V_SEPARATION 19
#define GALTON_TOP_Y 50
#define GALTON_BOTTOM_Y GALTON_TOP_Y + (GALTON_HEIGHT - 1) * GALTON_V_SEPARATION + PEG_RADIUS_INT

// Graph configuration
#define GRAPH_LEN GALTON_HEIGHT + 1
#define GRAPH_START_Y SCREEN_BOTTOM_Y
#define GRAPH_MAX_Y   GALTON_BOTTOM_Y + 10
#define GRAPH_BAR_WIDTH 30
#define MAX_PEGS GALTON_MAX_HEIGHT * GALTON_MAX_HEIGHT
#define PEG_RADIUS_INT 6
#define PEG_RADIUS_FIX int2fix15(PEG_RADIUS_INT)

// Peg array
static peg_t pegs[MAX_PEGS];
static int pegs_len = 0;

// Graph array
static int graph[GRAPH_LEN];

// Simulation parameters
static fix15 ball_gravity = float2fix15(0.75f);
static fix15 ball_bounciness = float2fix15(0.45f);
static float time_since_boot = 0;
static uint32_t num_fallen_through = 0;

// Number of samples per period in sine table
#define sine_table_size 256

// Sine table
int raw_sin[sine_table_size];

// Table of values to be sent to DAC
unsigned short DAC_data[sine_table_size];

// Pointer to the address of the DAC data table
unsigned short * _address_pointer = &DAC_data[0];

// DMA channels
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

// LED, button, and potentiometer configuration
#define LED 25
#define PIN_POT 27
#define POT_MAX 4000
#define POT_MIN_DEVIATION 40
#define BUTTON_PIN 10

// Potentiometer variables
static bool num_balls_set = true;
static int pot_last_value = 0;
static button_mode mode = BUTTON_MODE_NOTHING;
static button_state state = BUTTON_NOT_PRESSED;

// Number of DMA transfers per event
const uint32_t transfer_count = sine_table_size;

// Read the current value from the potentiometer
uint16_t readPotentiometer(void)
{
	return adc_read();
}

// Convert the button mode to a string
char *button_mode_to_str(button_mode _mode)
{
	switch (mode)
	{
		case BUTTON_MODE_NUM_BALLS: 
			return "Ball Count ";
		case BUTTON_MODE_BOUNCINESS: 
			return "Bounciness ";
		case BUTTON_MODE_GRAVITY: 
			return "Gravity    ";
		case BUTTON_MODE_NOTHING:
			return "Nothing    ";
	}
}

// Sequentialy change the button mode
void change_button_mode(void)
{
	switch (mode)
	{
		case BUTTON_MODE_NUM_BALLS:
			mode = BUTTON_MODE_BOUNCINESS;
			break;

		case BUTTON_MODE_BOUNCINESS:
			mode = BUTTON_MODE_GRAVITY;
			break;

		case BUTTON_MODE_GRAVITY:
			mode = BUTTON_MODE_NOTHING;
			break;
		
		case BUTTON_MODE_NOTHING:
			mode = BUTTON_MODE_NUM_BALLS;
			break;
	}
}

// Update the state of the button using the button state machine
void update_button_state(void)
{
	uint32_t pressed = (gpio_get(BUTTON_PIN) == 0);

	switch (state)
	{
		case BUTTON_NOT_PRESSED:
			if (pressed)
			{
				state = BUTTON_MAYBE_PRESSED;
			}
			break;

		case BUTTON_MAYBE_PRESSED:
			if (pressed)
			{
				// When the button enters the pressed state, change the button mode
				state = BUTTON_PRESSED;
				change_button_mode();
			}
			else
			{
				state = BUTTON_NOT_PRESSED;
			}
			break;

		case BUTTON_PRESSED:
			if (!pressed)
			{
				state = BUTTON_MAYBE_NOT_PRESSED;
			}
			break;

		case BUTTON_MAYBE_NOT_PRESSED:
			if (pressed)
			{
				state = BUTTON_PRESSED;
			}
			else
			{
				state = BUTTON_NOT_PRESSED;
			}
			break;
	}
}

// Reset a ball's position to the top of the screen
void resetBall(ball_t *ball)
{
	ball->last_peg = NULL;
	ball->x = int2fix15(BALL_START_X);
	ball->y = int2fix15(BALL_START_Y);
	ball->vy = 0;

	int r = rand() % 1000;
	r -= 500;
	
	ball->vx = multfix15(divfix(int2fix15(r), int2fix15(500)), int2fix15(BALL_START_DX_MAX));
}

// Create a new ball
void spawnball(void)
{
	ball_t *ball = &balls[balls_len++];
	resetBall(ball);
}

// Remove a ball from the screen
void removeball(ball_t *ball)
{
	drawCircle(fix2int15(ball->x), fix2int15(ball->y), BALL_RADIUS_INT, BLACK);
}

// Change the current number of balls in the simulation
void setNumBalls(uint32_t num_balls)
{
	if (num_balls > MAX_BALLS)
	{
		num_balls = MAX_BALLS;
	}

	// If the new amount of balls is greater than the old one, add new balls. Otherwise, remove balls
	if (balls_len < num_balls)
	{
		for (int i = balls_len; i < num_balls; i++)
		{
			spawnball();
		}
	}
	else if (balls_len > num_balls)
	{
		balls_len = num_balls;
	}
}

// Update the graph depending on the position of a ball leaving the galton board
void updateGraph(ball_t *ball)
{
	int startx = 320 - (((GALTON_HEIGHT-1)) * GALTON_H_SEPARATION)/2;
	int x = fix2int15(ball->x);

	if (x <= startx)
	{
		graph[0]++;
		return;
	}

	for (int j = 0; j <= GALTON_HEIGHT; j++)
	{
		if (x <= startx + j * GALTON_H_SEPARATION)
		{
			graph[j + 1]++;
			return;
		}
	}

	graph[GALTON_HEIGHT]++;
}

// Clear the displayed graph
void clearGraph(void)
{
	int startx = 320 - (((GALTON_HEIGHT+2)) * GALTON_H_SEPARATION)/2;

	for (int i = 0; i < GRAPH_LEN; i++)
	{
		drawRect(startx + i * GALTON_H_SEPARATION - GRAPH_BAR_WIDTH/2, GRAPH_START_Y - 100, GRAPH_BAR_WIDTH, 100, BLACK);
	}
}

// Draw the graph on the bottom of the screen
void drawGraph(void)
{
	int start_x = 320 - (int) (((float) (GALTON_HEIGHT + 2))/2.0 * GALTON_H_SEPARATION);
	int graph_max = 1;
	float proportion;
	int graph_height;

	for (int i = 0; i < GRAPH_LEN; i++)
	{
		if (graph[i] > graph_max)
		{
			graph_max = graph[i];
		}
	}

	for (int i = 0; i < GRAPH_LEN; i++)
	{
		if (graph[i] <= 0)
		{
			continue;
		}

		graph_height = fix2int15(multfix15(divfix(int2fix15(graph[i]),int2fix15(graph_max)), int2fix15(100)));

		fillRect(start_x + i * GALTON_H_SEPARATION - GRAPH_BAR_WIDTH/2, GRAPH_START_Y - 100, GRAPH_BAR_WIDTH, 100, BLACK);
		drawRect(start_x + i * GALTON_H_SEPARATION - GRAPH_BAR_WIDTH/2, GRAPH_START_Y - graph_height, GRAPH_BAR_WIDTH, graph_height, RED);
	}
}

// Reset the contents of the graph
void resetGraph(void)
{
	for (int i = 0; i < GRAPH_LEN; i++)
	{
		graph[i] = 0;
	}

	num_fallen_through = 0;

	clearGraph();
}

// Based on the potentiometer mode and value, update the simulation
void update_potentiometer(void)
{
	uint16_t pot_value = readPotentiometer();

	// Make sure the potentiometer has deviated enough for a valid update
	if (abs(pot_last_value - (int)pot_value) < POT_MIN_DEVIATION)
	{
		return;
	}

	pot_last_value = pot_value;

	if (pot_value > POT_MAX)
	{
		pot_value = POT_MAX;
	}

	switch (mode)
	{
		case BUTTON_MODE_NUM_BALLS:
			pot_value = (int) (((float) pot_value / (float) POT_MAX) * ((float) MAX_BALLS));
			setNumBalls(pot_value);
			break;

		case BUTTON_MODE_GRAVITY:
			ball_gravity = float2fix15(((float) pot_value / (float) POT_MAX) * (MAX_GRAVITY - MIN_GRAVITY) + MIN_GRAVITY);
			break;

		case BUTTON_MODE_BOUNCINESS:
			ball_bounciness = float2fix15(((float) pot_value / (float) POT_MAX) * (MAX_BOUNCINESS - MIN_BOUNCINESS) + MIN_BOUNCINESS);
			break;
		
		case BUTTON_MODE_NOTHING:
			break;
	}

	num_fallen_through = 0;
	resetGraph();
}

// Play the ball sound
void play_hit_sound(void)
{
	dma_channel_start(ctrl_chan);
}

// Find the nearest peg to a ball
peg_t *findPeg(ball_t *ball)
{
	// Find the row by dividing the ball's Y-coordinate by the Y separation of the pegs, relative to the top of the board
	int temp_y = fix2int15(ball->y);

	temp_y -= GALTON_TOP_Y;
	temp_y += GALTON_V_SEPARATION/2;
	
	int row = temp_y / GALTON_V_SEPARATION;

	// If outside of the board's y bounds, return no peg
	if (row < 0 || row >= GALTON_HEIGHT)
	{
		return NULL;
	}

	// Find the column by dividing the ball's X-coordinate by the X separation of the pegs, relative to the leftmost peg of the row
	int temp_x = ((row + 1) * GALTON_H_SEPARATION) / 2;
	int start_x = 320 - temp_x;
	int end_x = 320 + temp_x;
	
	// If outside of the row's x bounds, return no peg
	if (fix2int15(ball->x) > end_x || fix2int15(ball->x) < start_x)
	{
		return NULL;
	}

	temp_x = fix2int15(ball->x) - start_x;
	int col = temp_x / GALTON_H_SEPARATION;

	// Return the peg at the calculated index
	int index = row * (row + 1) / 2 + col;
	return &pegs[index];
}

// Update a ball's position and velocity, and detect any bounces. 
void updateBall(ball_t *ball)
{
	fix15 dx; 
	fix15 dy;
	fix15 dist; 
	fix15 normal_x;
	fix15 normal_y;
	fix15 temp;
	peg_t *peg;

	// Find the nearest peg
	peg = findPeg(ball);

	if (peg != NULL)
	{
		dx = ball->x - peg->x;
		dy = ball->y - peg->y;

		// check if the ball is overlapping the peg
		if (absfix15(dx) < BALL_RADIUS_FIX + PEG_RADIUS_FIX && absfix15(dy) < BALL_RADIUS_FIX + PEG_RADIUS_FIX)
		{
			// Use Alpha-max beta-min to estimate the distance
			fix15 absdx = absfix15(dx);
			fix15 absdy = absfix15(dy);

			fix15 max = MAX(absdx, absdy);
			fix15 min = MIN(absdx, absdy);

			dist = max + divfix(min, int2fix15(4));

			// Calculate the new velocity
			normal_x = divfix(dx, dist);
			normal_y = divfix(dy, dist);

			temp = multfix15(int2fix15(-2), (multfix15(normal_x, ball->vx) + multfix15(normal_y, ball->vy)));

			if (temp > 0)
			{
				ball->x = peg->x + multfix15(normal_x, dist + int2fix15(1));
				ball->y = peg->y + multfix15(normal_y, dist + int2fix15(1));

				ball->vx = ball->vx + multfix15(normal_x, temp);
				ball->vy = ball->vy + multfix15(normal_y, temp);

				// If hitting a new peg, play a hit sound
				if (peg != ball->last_peg)
				{
					ball->last_peg = peg;

					play_hit_sound();

					ball->vx = multfix15(ball->vx, ball_bounciness);
					ball->vy = multfix15(ball->vy, ball_bounciness);
				}
			}
		}
	}

	// Check if the balls are hitting the edge of the graph display and update their motion accordingly
	if (hitBottom(ball->y))
	{
		updateGraph(ball);
		num_fallen_through++;
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

	// Update the ball's velocity
	ball->vy += ball_gravity;

	ball->x += ball->vx;
	ball->y += ball->vy;
}

// Spawn a new peg at the given position
void spawnPeg(int x, int y)
{
	peg_t *peg = &pegs[pegs_len];
	peg->x = int2fix15(x);
	peg->y = int2fix15(y);
	pegs_len += 1;
}

// Create a galton board
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
void drawBoard() 
{
	for (int i = 0; i < pegs_len; i++)
	{
		drawCircle(fix2int15(pegs[i].x), fix2int15(pegs[i].y), 6, BLUE);
	}

	drawVLine(SCREEN_RIGHT_X, SCREEN_TOP_Y, SCREEN_BOTTOM_Y - SCREEN_TOP_Y, WHITE);
	drawVLine(SCREEN_LEFT_X, SCREEN_TOP_Y, SCREEN_BOTTOM_Y - SCREEN_TOP_Y, WHITE);
	drawHLine(SCREEN_LEFT_X, SCREEN_BOTTOM_Y, SCREEN_RIGHT_X - SCREEN_LEFT_X, WHITE);
	drawHLine(SCREEN_LEFT_X, SCREEN_TOP_Y, SCREEN_RIGHT_X - SCREEN_LEFT_X, WHITE);

	setCursor(10, 10);
	setTextSize(1);
	setTextColor2(WHITE, BLACK);
	
	static char str[256];

	sprintf(str, "Ball count:       %lu    \nBounciness:       %.02f  \nGravity:          %.02f     \nFallen through:   %lu   \nTime since boot:  %us   \nButton mode:      %s\n",
		 balls_len, fix2float15(ball_bounciness), fix2float15(ball_gravity), num_fallen_through, (int) time_since_boot,
		 button_mode_to_str(mode));

	// drawRect(10, 10, 200, 100, BLACK);
	writeString(str);
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

	for (int i = 0; i < MIN_BALLS; i++)
	{
		spawnball();
	}

	createGalton();

	while(1) 
	{
		// TODO: handle multiple balls and pegs
		// Measure time at start of thread
		begin_time = time_us_32();    
		
		for (int i = 0; i < balls_len; i++)
		{
			// erase ball
			drawCircle(fix2int15(balls[i].x), fix2int15(balls[i].y), BALL_RADIUS_INT, BLACK);
		}

		update_potentiometer();
		
		for (int i = 0; i < balls_len; i++)
		{
			// update ball's position and velocity, check for peg hits
			updateBall(&balls[i]);

			drawCircle(fix2int15(balls[i].x), fix2int15(balls[i].y), BALL_RADIUS_INT, GREEN);
			// drawRect(fix2int15(balls[i]->x) - BALL_RADIUS_INT, fix2int15(balls[i]->y) - BALL_RADIUS_INT, BALL_RADIUS_INT * 2 + 1, BALL_RADIUS_INT * 2 + 1, GREEN);
			// draw the ball at its new position
		}

		// draw the boundaries
		drawBoard();

		drawGraph();

		readPotentiometer();
		
		update_button_state();

		// delay in accordance with frame rate
		spare_time = FRAME_RATE - (time_us_32() - begin_time);
		// yield for necessary amount of time

		gpio_put(LED, spare_time < 0);

		PT_YIELD_usec(spare_time);
		time_since_boot += ((float) FRAME_RATE) / ((float) 1000000);
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

	// Initialize ADC
	adc_init();

    // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports, acts like framed SPI with this CS mapping
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

	gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 0);

	adc_gpio_init(PIN_POT);
	adc_select_input(1);

	gpio_set_dir(BUTTON_PIN, false);
	gpio_pull_up(BUTTON_PIN);

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
