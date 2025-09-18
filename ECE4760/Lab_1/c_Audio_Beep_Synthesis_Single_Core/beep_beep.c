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
#include "hardware/spi.h"
#include "hardware/sync.h"
// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"

// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

/**
 * Used to track the state of the keypad
 */
typedef enum _keypad_state {
    KEYPAD_NOT_PRESSED,
    KEYPAD_MAYBE_PRESSED,
    KEYPAD_PRESSED,
    KEYPAD_MAYBE_NOT_PRESSED
} keypad_state;

/**
 * Used to track the current sound, and record previous sounds
 */
typedef enum _sound {
    SOUND_CHIRP,
    SOUND_SWOOP,
    SOUND_SILENCE,
    SOUND_BIRD_NOISE_1,
    SOUND_BIRD_NOISE_2,
    SOUND_BIRD_NOISE_3,
    SOUND_NONE
} sound;

/**
 * Used to track the playback mode
 */
typedef enum _playback_mode {
    MODE_RECORD,
    MODE_PLAY
} playback_mode;

/**
 * Used to record the played audio
 */
typedef struct _recorded_sound {
    sound sound;
    uint32_t timestamp;
} recorded_sound;

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
#define SWOOP_LEN 6500
#define CHIRP_LEN 6500
#define SILENCE_LEN 3000
#define BIRD_NOISE_1_LEN 10000
#define BIRD_NOISE_2_LEN 5000
#define BIRD_NOISE_3_LEN 3112
static fix15 swoop[SWOOP_LEN] = {0};
static fix15 chirp[CHIRP_LEN] = {0};
static fix15 silence[SILENCE_LEN] = {0};
static fix15 bird_noise_1[BIRD_NOISE_1_LEN] = {0};
static fix15 bird_noise_2[BIRD_NOISE_2_LEN] = {0};
static fix15 bird_noise_3[BIRD_NOISE_3_LEN] = {0};
static recorded_sound recorded_sounds[256];
static uint32_t recorded_sounds_len = 0;
static uint32_t record_playback_index = 0;
static playback_mode mode = MODE_PLAY;

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

#define BASE_KEYPAD_PIN 9
#define KEYROWS         4
#define NUMKEYS         12

//GPIO for timing the ISR
#define ISR_GPIO 7

#define BASE_KEYPAD_PIN 9

/**
 * Return the length of the input sound, measured in interrupts
 */
inline uint32_t sound_length(sound sound)
{
    switch (sound)
    {
        case SOUND_CHIRP:
            return CHIRP_LEN;
        case SOUND_SWOOP:
            return SWOOP_LEN;
        case SOUND_SILENCE:
            return SILENCE_LEN;
        case SOUND_BIRD_NOISE_1:
            return BIRD_NOISE_1_LEN;
        case SOUND_BIRD_NOISE_2:
            return BIRD_NOISE_2_LEN;
        case SOUND_BIRD_NOISE_3:
            return BIRD_NOISE_3_LEN;
        case SOUND_NONE:
            return 0;
        default:
            return SWOOP_LEN;
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
        case SOUND_CHIRP:
            frequencies = chirp;
            break;
        case SOUND_SWOOP:
            frequencies = swoop;
            break;
        case SOUND_SILENCE:
            frequencies = silence;
            break;
        case SOUND_BIRD_NOISE_1:
            frequencies = bird_noise_1;
            break;
        case SOUND_BIRD_NOISE_2:
            frequencies = bird_noise_2;
            break;
        case SOUND_BIRD_NOISE_3:
            frequencies = bird_noise_3;
            break;
        case SOUND_NONE:
            frequencies = swoop;
            break;
        
    }

    // Access the sound's frequency table based on the frequency index
    return (int) (((uint32_t) fix2int15(frequencies[count])) * (two32/Fs));
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
        DAC_data_0 = (DAC_config_chan_B) | (DAC_output_0 & 0xffff);

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1);

        // Increment the counter
        count_0 += 1;

        // State transition?
        if (count_0 == sound_length(current_sound)) 
        {
            if (mode == MODE_RECORD)
            {
                STATE_0 = 1;
                count_0 = 0;
            }
            else
            {
                // If there are more recorded sounds, play the next one
                if (++record_playback_index < recorded_sounds_len)
                {
                    current_sound = recorded_sounds[record_playback_index].sound;
                    STATE_0 = 0;
                }
                else
                {
                    STATE_0 = 1;
                }

                phase_accum_main_0 = 0;
                current_amplitude_0 = 0;
                count_0 = 0;
            }
        }
    }

    // De-assert the GPIO when we leave the interrupt
    gpio_put(ISR_GPIO, 0);

}

// Returns the currently pressed key, or -1 if the keypad is in an invalid state
static int get_pressed_key()
{
    unsigned int keycodes[NUMKEYS] = {   0x28, 0x11, 0x21, 0x41, 0x12,
                                        0x22, 0x42, 0x14, 0x24, 0x44,
                                        0x18, 0x48};
    unsigned int scancodes[KEYROWS] = {   0x01, 0x02, 0x04, 0x08};
    unsigned int button = 0x70;
    int keypad = 0x00;

    // Scan the keypad!
    for (int i = 0; i < KEYROWS; i++) 
    {
        // Set a row high
        gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                        (scancodes[i] << BASE_KEYPAD_PIN));

        // Small delay required
        sleep_us(1); 

        // Read the keycode
        keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F);

        // Look for a valid keycode if button(s) are pressed
        if (keypad & button) 
        {
            for (int i = 0; i < NUMKEYS; i++) 
            {
                if (keypad == keycodes[i]) 
                {
                    return i;
                }
            }

            return -1;
        }
    }

    return -1;
}

// This thread runs on core 0
static PT_THREAD (protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt);
    while(1) 
    {
        static keypad_state state = KEYPAD_NOT_PRESSED;

        // Toggle on LED
        gpio_put(LED, !gpio_get(LED));

        int key = get_pressed_key();

        // The keypad state machine
        switch (state)
        {
            // If in "not pressed" and a key is pressed, switch to "maybe pressed"
            case KEYPAD_NOT_PRESSED:
                if (key == -1)
                {
                    state = KEYPAD_NOT_PRESSED;
                }
                else
                {
                    state = KEYPAD_MAYBE_PRESSED;
                }
                break;
            
            /**
             * If in "maybe pressed" and a key is pressed, switch to "pressed". Otherwise, switch to "not pressed"
             * When switching to pressed, handle the key press.
             */ 
            case KEYPAD_MAYBE_PRESSED:
                if (key == -1)
                {
                    state = KEYPAD_NOT_PRESSED;
                }
                else
                {
                    state = KEYPAD_PRESSED;

                    printf("pressed key: %d\n", key);

                    /**
                     * Handle the key press:
                     * 
                     * 1 - play "swoop"
                     * 2 - play "chirp"
                     * 3 - play "silence"
                     * 6 - play "bird noise 1"
                     * 6 - play "bird noise 2"
                     * 6 - play "bird noise 3"
                     * 0 - switch modes
                     */
                    switch (key)
                    {
                        case 1:
                            current_sound = SOUND_SWOOP;
                            break;
                        case 2:
                            current_sound = SOUND_CHIRP;
                            break;
                        case 3:
                            current_sound = SOUND_SILENCE;
                            break;
                        case 4:
                            current_sound = SOUND_BIRD_NOISE_1;
                            break;
                        case 5:
                            current_sound = SOUND_BIRD_NOISE_2;
                            break;
                        case 6:
                            current_sound = SOUND_BIRD_NOISE_3;
                            break;
                        case 0:
                            mode = (mode == MODE_PLAY) ? MODE_RECORD : MODE_PLAY;

                            printf("Mode changed to %s mode\n", mode == MODE_PLAY ? "play" : "record");

                            // Reset DDS and start playback when switching to play mode
                            if (mode == MODE_PLAY)
                            {
                                record_playback_index = 0;
                                current_sound = recorded_sounds[0].sound;
                                phase_accum_main_0 = 0;
                                current_amplitude_0 = 0;
                                STATE_0 = 0;
                                count_0 = 0;
                            }
                            else
                            {
                                recorded_sounds_len = 0;
                            }

                            break; 

                        default:
                            current_sound = SOUND_SILENCE;
                            break;
                    }

                    // If pressing a sound key, reset DDS
                    if (key != 0)
                    {
                        phase_accum_main_0 = 0;
                        current_amplitude_0 = 0;
                        STATE_0 = 0;
                        count_0 = 0;

                        if (mode == MODE_RECORD)
                        {
                            recorded_sounds[recorded_sounds_len++].sound = current_sound;
                            // recorded_sounds[recorded_sounds_len].timestamp = current_sound;
                        }
                    }
                }
                break;
            
            // If in "pressed" and no key is pressed, switch to "maybe not pressed"
            case KEYPAD_PRESSED:
                if (key == -1)
                {
                    state = KEYPAD_MAYBE_NOT_PRESSED;
                }
                else
                {
                    state = KEYPAD_PRESSED;
                }
                
                break;
            
            // If in "maybe not pressed" and no key is pressed, switch to "not pressed". Otherwise, switch back to pressed.
            case KEYPAD_MAYBE_NOT_PRESSED:
                if (key == -1)
                {
                    state = KEYPAD_NOT_PRESSED;
                    printf("pressed key: none\n");
                }
                else
                {
                    state = KEYPAD_PRESSED;
                }

                break;

            default:
                break;
        }

        // Yield for 10 ms
        PT_YIELD_usec(10000);
    }
    // Indicate thread end
    PT_END(pt);
}

// Core 0 entry point
int main() 
{
    // Initialize stdio/uart (printf won't work unless you do this!)
    stdio_init_all();
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
    
    // Initialize the keypad GPIO's
    gpio_init_mask((0x7F << BASE_KEYPAD_PIN));
    // Set row-pins to output
    gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN));
    // Set all output pins to low
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0x0 << BASE_KEYPAD_PIN));
    // Turn on pulldown resistors for column pins (on by default)
    gpio_pull_down((BASE_KEYPAD_PIN + 4));
    gpio_pull_down((BASE_KEYPAD_PIN + 5));
    gpio_pull_down((BASE_KEYPAD_PIN + 6));

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
    for (x = 0; x < SWOOP_LEN; x++)
    {
        float frequency = (-260 * (float) sin((((float) -3.141592)/((float)SWOOP_LEN))*x)) + 1740;
        swoop[x] = float2fix15(frequency);
    }

    // Create a frequency table for chirp
    for (x = 0; x < CHIRP_LEN; x++)
    {
        float frequency = .000184 * x * x + 2000;
        chirp[x] = float2fix15(frequency);
    }

    // Create a frequency table for silence
    for (x = 0; x < SILENCE_LEN; x++)
    {
        float frequency = 0;
        silence[x] = float2fix15(frequency);
    }

    // Create a frequency table for bird noise 1
    for (x = 0; x < BIRD_NOISE_1_LEN; x++)
    {
        float frequency = 2750 + (0.0066 * (x - 11000)) * (0.0066 * (x - 11000));
        bird_noise_1[x] = float2fix15(frequency);
    }

    // Create a frequency table for bird noise 2
    for (x = 0; x < BIRD_NOISE_2_LEN; x++)
    {
        float frequency = 2600 - 0.1 * x;
        bird_noise_2[x] = float2fix15(frequency);
    }

    // Create a frequency table for bird noise 3
    for (x = 0; x < BIRD_NOISE_3_LEN; x++)
    {
        float frequency = -0.0003238 * (x - 1556) * (x - 1556) + 2000;
        bird_noise_3[x] = float2fix15(frequency);
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

    // Start scheduling core 0 threads
    pt_schedule_start;

}
