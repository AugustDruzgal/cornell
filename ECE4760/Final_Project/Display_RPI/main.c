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

// Physics constants - not all used in the final version
static const float BOX_GRAVITY         = 80.0f;   // pixels/s^2, downward (screen)
static const float BOX_MASS            = 12.0f;
static const float BOARD_INERTIA       = 1200.0f;
static const float BOARD_SPRING_K      = 15.0f;
static const float BOARD_DAMP          = 1.5f;
static const float STATIC_WEIGHT_SCALE = 14.0f;

// Grid sliding parameters - not all used in the final version
static const float GRID_SLIDE_SCALE = 0.4f;   // how strongly gravity along board moves pile
static const float GRID_FRICTION    = 0.99f;  // 0<GRID_FRICTION<=1, lower = more damping

// Time step (matches PT_YIELD_usec(16000))
static const float PHYSICS_DT = 0.016f;

// The angle of the device
static float tilt_angle = 0.00f;

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
static const int   MAX_HEALTH_POINTS = 20;
static int         health_points     = 20;
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

static inline void drawBoardPaddle(float c, float s) {
    drawRotatedRectOutline(BOARD_X, BOARD_Y, BOARD_WIDTH, BOARD_HEIGHT,
                           c, s, WHITE);
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
            rand_color = ORANGE;
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

static void physics_step(float c, float s) {
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

    // This will need to be changed to a torque
    pplat.angle = tilt_angle * DEGtoRAD;

    // Clamp board angle to +/- 30 degrees
    const float max_angle_rad = 30.0f * DEGtoRAD;
    if (pplat.angle >  max_angle_rad) pplat.angle =  max_angle_rad;
    if (pplat.angle < -max_angle_rad) pplat.angle = -max_angle_rad;

    board_angle_deg = pplat.angle * DEGtoRAD;

    // 5) CLEANUP GRID CELLS THAT SLID OFF-SCREEN
    cleanup_grid(c, s);
}

// ---------------------------------------------------------
// 6. PROTOTHREAD: VGA DRAW
// ---------------------------------------------------------

static PT_THREAD (protothread_vga(struct pt *pt))
{
    PT_BEGIN(pt);

    physics_init();

    setTextSize(1);
    setTextColor2(WHITE, BLACK);

    while (true) {

        float c     = cosf(pplat.angle);
        float s     = sinf(pplat.angle);
        // Step physics
        physics_step(c, s);

        // Clear screen
        // fillRect(0, 0, 640, 480, BLACK);
        // Clear only the play area, not the whole 640x480
        fillRect(100, 0, 530, 450, BLACK); //More efficient way to do this?


        // Optional borders
        // drawBoardBounds();

        // Draw snapped grid boxes (white)
        

        // Draw board
        drawBoardPaddle(c, s);

        
        float halfL = 0.5f * (float)BOARD_WIDTH;
        float halfH = 0.5f * (float)BOARD_HEIGHT;

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

        // ----------------------------
        // Health bar + Timer
        // ----------------------------
        char buf[64];

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
        sprintf(buf, "Time: %.1fs", elapsed_s);
        writeString(buf);
        
        setCursor(SCREEN_LEFT_X + 10, SCREEN_TOP_Y + 20);
        sprintf(buf, "Angle: %.1f deg", tilt_angle);
        writeString(buf);

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
        sprintf(buf, "HP: %2d", health_points);
        writeString(buf);

        // Game over message
        if (game_over) {
            setCursor(SCREEN_RIGHT_X - 140, SCREEN_BOTTOM_Y - 40);
            writeString("YOU LOSE!");

            setCursor(SCREEN_RIGHT_X - 220, SCREEN_BOTTOM_Y - 20);
            sprintf(buf, "Time spent in game: %.1fs", final_time_s);
            writeString(buf);
        }

        
        // ~60 fps
        PT_YIELD_usec(16000);
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

    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    bt_client_main();

    // Nothing on core 0 (all work is on core 1)
    while (1) {
        tight_loop_contents();
    }

    return 0;
}


