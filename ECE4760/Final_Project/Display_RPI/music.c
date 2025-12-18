/**
 *  V. Hunter Adams (vha3@cornell.edu)
 
    A timer interrupt on core 0 generates a 400Hz beep
    thru an SPI DAC, once per second. A single protothread
    blinks the LED.

    GPIO 5 (pin 7) Chip select
    GPIO 6 (pin 9) SCK/spi0_sclk
    GPIO 7 (pin 10) MOSI/spi0_tx
    GPIO 2 (pin 4) GPIO output for timing ISR
    3.3v (pin 36) -> VCC on DAC 
    GND (pin 3)  -> GND on DAC 

 */

// Include necessary libraries
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include <stdint.h>
#include "hardware/spi.h"
#include "hardware/sync.h"
// Include protothreads
#include "pt_cornell_rp2040_v1_4_client.h"
//#include "menu_song.c"
//#include "game_song.c"

// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

/**
 * Used to track the current sound, and record previous sounds
 */
typedef enum _sound {
    SOUND_NOTE,
    SOUND_NONE
} sound;

// Macros for fixed-point arithmetic (faster than floating point)
typedef signed int fix15;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) 
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)( (((signed long long)(a)) << 15) / (b))

/**
 * Used to record the played audio
 */
typedef struct _song_note {
    fix15 frequency;
    uint32_t delay;
} song_note;

typedef enum _songs {
    SONG_GAME,
    SONG_MENU
} song;

//Direct Digital Synthesis (DDS) parameters
#define two32 4294967296.0  // 2^32 (a constant)
#define Fs 50000
#define DELAY 20 // 1/Fs (in microseconds)

// the DDS units - core 0
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_0;
volatile unsigned int phase_incr_main_0 = (400.0*two32)/Fs;

// DDS sine table (populated in main())
#define sine_table_size 256
fix15 sin_table[sine_table_size];
#define NOTE_LEN 8000
static fix15 note[NOTE_LEN] = {0};
static uint32_t recorded_sounds_len = 0;
static uint32_t record_playback_index = 0;
static song_note menu_song[256] = {
    {
        .frequency = float2fix15(0.6),
        .delay = 250000,
    },
    {
        .frequency = float2fix15(0.7),
        .delay = 250000,
    },
    {
        .frequency = float2fix15(0.8),
        .delay = 250000,
    },
    {
        .frequency = float2fix15(0.9),
        .delay = 250000,
    },
    {
        .frequency = float2fix15(1.0),
        .delay = 250000,
    },
    {
        .frequency = float2fix15(1.1),
        .delay = 250000,
    },
    {
        .frequency = float2fix15(1.2),
        .delay = 250000,
    },
    {
        .frequency = float2fix15(1.3),
        .delay = 250000,
    },
};

static song_note game_song[256] = {
    {
        .frequency = float2fix15(0.7),
        .delay = 250000,
    },
    {
        .frequency = float2fix15(0.9),
        .delay = 250000,
    },
    {
        .frequency = float2fix15(0.7),
        .delay = 250000,
    },
    {
        .frequency = float2fix15(0.9),
        .delay = 250000,
    },
    {
        .frequency = float2fix15(0.7),
        .delay = 250000,
    },
    {
        .frequency = float2fix15(1.1),
        .delay = 250000,
    },
    {
        .frequency = float2fix15(0.7),
        .delay = 250000,
    },
    {
        .frequency = float2fix15(1.1),
        .delay = 250000,
    },
};

#define MENU_SONG_LEN 8

static song_note *current_song = menu_song;
static int song_index = 0;

// Values output to DAC
int DAC_output_0;
int DAC_output_1;

// Amplitude modulation parameters and variables
fix15 max_amplitude = int2fix15(1);    // maximum amplitude
fix15 attack_inc;                      // rate at which sound ramps up
fix15 decay_inc;                       // rate at which sound ramps down
fix15 current_amplitude_0 = 0;         // current amplitude (modified in ISR)
fix15 current_amplitude_1 = 0;         // current amplitude (modified in ISR)

// Timing parameters for beeps (units of interrupts)
#define ATTACK_TIME             250
#define DECAY_TIME              250
#define SUSTAIN_TIME            10000
#define BEEP_DURATION           10500
#define BEEP_REPEAT_INTERVAL    50000

// State machine variables
volatile unsigned int STATE_0 = 1;
volatile unsigned int count_0 = 0;
static sound current_sound;
static fix15 frequency_mult = int2fix15(1);

// SPI data
uint16_t DAC_data_1; // output value
uint16_t DAC_data_0; // output value

// DAC parameters (see the DAC datasheet)
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

//SPI configurations (note these represent GPIO number, NOT pin number)
#define PIN_SCK  2
#define PIN_MISO 3
#define PIN_MOSI 4
#define PIN_CS   5
#define LDAC     9
#define LED      25
#define SPI_PORT spi0

//GPIO for timing the ISR
#define ISR_GPIO 7

/**
 * Return the length of the input sound, measured in interrupts
 */
inline uint32_t sound_length(sound sound)
{
    switch (sound)
    {
        case SOUND_NOTE:
            return NOTE_LEN;
        case SOUND_NONE:
            return 0;
        default:
            return NOTE_LEN;
    }
}

/**
 * Return the DDS increment based on the sound and frequency index for the sound
 */
static int increment_from_sound(sound sound, int count)
{
    fix15 *frequencies;

    switch (sound)
    {
        case SOUND_NOTE:
            frequencies = note;
            break;
        case SOUND_NONE:
            frequencies = note;
            break;
    }

    // Access the sound's frequency table based on the frequency index
    return (int) (((uint32_t) fix2int15(multfix15(frequencies[count], frequency_mult))) * (two32/Fs));
}

// This timer ISR is called on core 0
static void alarm_irq(void) 
{
    // Assert a GPIO when we enter the interrupt
    gpio_put(ISR_GPIO, 1);

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Reset the alarm register
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    if (STATE_0 == 0) 
    {
        // DDS phase and sine table lookup
        
        int DDS_increment;

        DDS_increment = increment_from_sound(current_sound, count_0);

        phase_accum_main_0 += DDS_increment;

        // printf("%d, %d\n", DDS_increment, fix2int15(swoop[count_0]));

        DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
            sin_table[phase_accum_main_0>>24])) + 2048;

        // Ramp up amplitude
        if (count_0 < ATTACK_TIME) 
        {
            current_amplitude_0 = (current_amplitude_0 + attack_inc);
        }
        // Ramp down amplitude
        else if (count_0 > sound_length(current_sound) - DECAY_TIME) 
        {
            current_amplitude_0 = (current_amplitude_0 - decay_inc);
        }

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_A) | (DAC_output_0 & 0xffff);

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1);

        // Increment the counter
        count_0 += 1;

        // State transition?
        if (count_0 == sound_length(current_sound)) 
        {
            STATE_0 = 1;
            count_0 = 0;
        }
    }

    // De-assert the GPIO when we leave the interrupt
    gpio_put(ISR_GPIO, 0);

}

static void play_note(fix15 mult)
{
    current_sound = SOUND_NOTE;
    STATE_0 = 0;
    phase_accum_main_0 = 0;
    current_amplitude_0 = 0;
    count_0 = 0;
    frequency_mult = mult;
}

static void set_song (song s)
{
    if (s == SONG_MENU)
    {
        current_song = menu_song;
        song_index = 0;
    }
    else if (s == SONG_GAME)
    {
        current_song = game_song;
        song_index = 0;
    }
    else
    {
        current_song = NULL;
    }
}

// This thread runs on core 0
static PT_THREAD (protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt);
    
    static float frequency = 1.0f;

    while(1) 
    {
        play_note(current_song[song_index].frequency);

        // Yield for 1s
        PT_YIELD_usec(current_song[song_index].delay);

        song_index += 1;

        if (song_index >= MENU_SONG_LEN)
        {
            song_index = 0;
        }
    }
    // Indicate thread end
    PT_END(pt);
}

// Core 0 entry point
int music_init(void) 
{
    printf("Hello, friends!\n");

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000);
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);

    // Map LDAC pin to GPIO port, hold it low (could alternatively tie to GND)
    gpio_init(LDAC);
    gpio_set_dir(LDAC, GPIO_OUT);
    gpio_put(LDAC, 0);

    // Setup the ISR-timing GPIO
    gpio_init(ISR_GPIO);
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0);

    // Map LED to GPIO port, make it low
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 0);

    // set up increments for calculating bow envelope
    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME));
    decay_inc =  divfix(max_amplitude, int2fix15(DECAY_TIME));

    // Build the sine lookup table
    // scaled to produce values between 0 and 4096 (for 12-bit DAC)
    int x;
    for (x = 0; x < sine_table_size; x++)
    {
         sin_table[x] = float2fix15(2047*sin((float)x*6.283/(float)sine_table_size));
    }

    // Create a frequency table for swoop
    for (x = 0; x < NOTE_LEN; x++)
    {
        if (x < NOTE_LEN/3)
        {
            int sqr = NOTE_LEN/3-x;
            float frequency = 440 - 0.000005 * sqr * sqr;
            note[x] = float2fix15(frequency);
        }
        else
        {
            float frequency = 440;
            note[x] = float2fix15(frequency);
        }
    }

    // Enable the interrupt for the alarm (we're using Alarm 0)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    // Associate an interrupt handler with the ALARM_IRQ
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    // Enable the alarm interrupt
    irq_set_enabled(ALARM_IRQ, true);
    // Write the lower 32 bits of the target time to the alarm register, arming it.
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    // Add core 0 threads
    pt_add_thread(protothread_core_0);
}
