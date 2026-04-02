// =====================================================
// USER CONFIGURATION - DEFAULT VALUES
// This is the distribution template with factory defaults.
// Copy to general_config.user.h and customize for your machine.
// =====================================================

// =====================================================
// DEBUG AND SENSOR TUNING
// =====================================================
#define DEBUG_TURRET 0          // Set to 0 to disable turret serial prints

// Hardware vs. Software Configuration Flag for accurate no-miss ball sensors
// Set to 0: Uses software polling. Ignores EMI noise, no hardware mods needed.
// Set to 1: Uses hardware interrupts. ONLY use if you have installed physical RC filters to stop EMI.
#define USE_HARDWARE_INTERRUPTS 0 

// =====================================================
// SCOREMORE SERIAL BAUD RATE & CONFIG
// =====================================================
#ifndef SCOREMORE_BAUD
#define SCOREMORE_BAUD      9600  // DEFAULT: 9600
#endif
#ifndef SCOREMORE_USER
#define SCOREMORE_USER      0     // DEFAULT: 0, do not wait for Scoremore connect. Set to 1 to wait for Scoremore before intialization.
#endif

// =====================================================
// LED HARDWARE TOGGLE
// =====================================================
// Uncomment to drive NeoPixels natively from the Arduino Mega.
// Comment out to use external WLED ESP32 via Serial1.
// #define USE_NATIVE_LEDS // using WLED ESP32 driver

#ifndef DECK_LED_LENGTH_L
#define DECK_LED_LENGTH_L   11    // Left deck strip.  DEFAULT: 11
#endif
#ifndef DECK_LED_LENGTH_R
#define DECK_LED_LENGTH_R   11    // Right deck strip. DEFAULT: 11
#endif
#ifndef LANE_LED_LENGTH_L
#define LANE_LED_LENGTH_L   41    // Left lane strip.  DEFAULT: 41
#endif
#ifndef LANE_LED_LENGTH_R
#define LANE_LED_LENGTH_R   41    // Right lane strip. DEFAULT: 41
#endif

// =====================================================
// WLED PRESET MAPPING CONFIGURATION
// =====================================================
#ifndef WLED_PRESET_NORMAL
#define WLED_PRESET_NORMAL      1   
#endif
#ifndef WLED_PRESET_BOOTING
#define WLED_PRESET_BOOTING     2   
#endif
#ifndef WLED_PRESET_MAINTENANCE
#define WLED_PRESET_MAINTENANCE 3   
#endif
#ifndef WLED_PRESET_STRIKE
#define WLED_PRESET_STRIKE      4   
#endif
#ifndef WLED_PRESET_SPARE
#define WLED_PRESET_SPARE       5  
#endif
#ifndef WLED_PRESET_PAUSE
#define WLED_PRESET_PAUSE       6   
#endif
#ifndef WLED_PRESET_THROW
#define WLED_PRESET_THROW       7   
#endif
#ifndef WLED_PRESET_FILL
#define WLED_PRESET_FILL        8   
#endif
#ifndef WLED_PRESET_BALL1
#define WLED_PRESET_BALL1       9   
#endif
#ifndef WLED_PRESET_BALL2
#define WLED_PRESET_BALL2      10   
#endif
#ifndef WLED_PRESET_MISS
#define WLED_PRESET_MISS       11   
#endif

// =====================================================
// CONVEYOR SETTINGS
// =====================================================
#ifndef CONVEYOR_ACTIVE_HIGH
#define CONVEYOR_ACTIVE_HIGH 1    // DEFAULT: 1
#endif
#ifndef IR_PIN_LOCKOUT_MS
#define IR_PIN_LOCKOUT_MS 300   // DEFAULT: 300ms 
#endif

// =====================================================
// LED BRIGHTNESS (0-255)
// =====================================================
#ifndef LED_BRIGHTNESS_NORMAL
#define LED_BRIGHTNESS_NORMAL  80 
#endif
#ifndef DECK_LED_BRIGHTNESS
#define DECK_LED_BRIGHTNESS    80 
#endif
#ifndef LED_BRIGHTNESS_STRIKE
#define LED_BRIGHTNESS_STRIKE  40 
#endif

// =====================================================
// SWEEP SERVO ANGLES
// =====================================================
#ifndef SWEEP_GUARD_ANGLE
#define SWEEP_GUARD_ANGLE   50    
#endif
#ifndef SWEEP_UP_ANGLE
#define SWEEP_UP_ANGLE      85    
#endif
#ifndef SWEEP_BACK_ANGLE
#define SWEEP_BACK_ANGLE    0     
#endif
#ifndef SWEEP_TWEEN_MS
#define SWEEP_TWEEN_MS      500   
#endif

// =====================================================
// BALL DOOR SERVO ANGLES & TIMING
// =====================================================
#ifndef BALL_DOOR_OPEN_ANGLE
#define BALL_DOOR_OPEN_ANGLE    180  
#endif
#ifndef BALL_DOOR_CLOSED_ANGLE
#define BALL_DOOR_CLOSED_ANGLE  0    
#endif
#ifndef BALL_DOOR_WAIT_MS
#define BALL_DOOR_WAIT_MS       1700  
#endif
#ifndef BALL_DOOR_OPEN_MS
#define BALL_DOOR_OPEN_MS       10000 
#endif

// =====================================================
// SCISSOR SERVO ANGLES
// =====================================================
#ifndef SCISSOR_GRAB_ANGLE
#define SCISSOR_GRAB_ANGLE  140   
#endif
#ifndef SCISSOR_DROP_ANGLE
#define SCISSOR_DROP_ANGLE  90    
#endif

// =====================================================
// SLIDING DECK SERVO ANGLES
// =====================================================
#ifndef SLIDER_HOME_ANGLE
#define SLIDER_HOME_ANGLE       180  
#endif
#ifndef SLIDER_RELEASE_ANGLE
#define SLIDER_RELEASE_ANGLE    100  
#endif

// =====================================================
// RAISE SERVO ANGLES
// =====================================================
#ifndef RAISE_UP_ANGLE
#define RAISE_UP_ANGLE          180  
#endif
#ifndef RAISE_DOWN_ANGLE
#define RAISE_DOWN_ANGLE        20   
#endif
#ifndef RAISE_GRAB_ANGLE
#define RAISE_GRAB_ANGLE        60   
#endif
#ifndef RAISE_DROP_ANGLE
#define RAISE_DROP_ANGLE        80   
#endif

// =====================================================
// TURRET STEPPER SETTINGS
// =====================================================
#ifndef TURRET_NORMAL_MAXSPEED
#define TURRET_NORMAL_MAXSPEED  650.0
#endif
#ifndef TURRET_NORMAL_ACCEL
#define TURRET_NORMAL_ACCEL     3000.0
#endif
#ifndef TURRET_SPRING_MAXSPEED
#define TURRET_SPRING_MAXSPEED  300.0
#endif
#ifndef TURRET_SPRING_ACCEL
#define TURRET_SPRING_ACCEL     1500.0
#endif
#ifndef TURRET_HOME_ADJUSTER
#define TURRET_HOME_ADJUSTER    -63     
#endif
#ifndef TURRET_PIN10_RELEASE_OFFSET
#define TURRET_PIN10_RELEASE_OFFSET -30
#endif
#ifndef TURRET_EMPTY_EXTRA_OFFSET
#define TURRET_EMPTY_EXTRA_OFFSET   -60 
#endif

// =====================================================
// TURRET PIN POSITIONS (steps from home)
// =====================================================
#ifndef PIN_POS_0
#define PIN_POS_0       0
#endif
#ifndef PIN_POS_1
#define PIN_POS_1       -133
#endif
#ifndef PIN_POS_2
#define PIN_POS_2       -267
#endif
#ifndef PIN_POS_3
#define PIN_POS_3       -400
#endif
#ifndef PIN_POS_4
#define PIN_POS_4       -667
#endif
#ifndef PIN_POS_5
#define PIN_POS_5       -800
#endif
#ifndef PIN_POS_6
#define PIN_POS_6       -933
#endif
#ifndef PIN_POS_7
#define PIN_POS_7       -1200
#endif
#ifndef PIN_POS_8
#define PIN_POS_8       -1333
#endif
#ifndef PIN_POS_9
#define PIN_POS_9       -1467
#endif
#ifndef PIN_POS_10
#define PIN_POS_10      -1600
#endif

// =====================================================
// INPUT DEBOUNCE TIME (milliseconds)
// =====================================================
#ifndef DEBOUNCE_MS
#define DEBOUNCE_MS     10    // Changed from 50. Required to catch fast gravity drop.
#endif

// =====================================================
// TURRET TIMING (milliseconds)
// =====================================================
#ifndef CATCH_DELAY_MS
#define CATCH_DELAY_MS          150   // Changed from 800. Time allowed for gravity to clear the pin.
#endif

// -- ORPHANED VARIABLES FOR CLOSED-LOOP FORK --
//#ifndef RELEASE_DWELL_MS
//#define RELEASE_DWELL_MS        1000  
//#endif
//#ifndef RELEASE_FEED_ASSIST_MS
//#define RELEASE_FEED_ASSIST_MS  250   
//#endif
//#ifndef NINTH_SETTLE_MS
//#define NINTH_SETTLE_MS         300   
//#endif
// ---------------------------------------------

// =====================================================
// EVERYTHING-SPECIFIC TIMING (milliseconds)
// =====================================================
#ifndef DECK_EXTRA_SETTLE_MS
#define DECK_EXTRA_SETTLE_MS    500   
#endif
#ifndef STRIKE_WIPE_MS
#define STRIKE_WIPE_MS          300   
#endif
#ifndef STRIKE_FRAME_MS
#define STRIKE_FRAME_MS         15    
#endif
#ifndef STRIKE_SWEEP_PAUSE_MS
#define STRIKE_SWEEP_PAUSE_MS   1000  
#endif
#ifndef BALL_COMET_MS
#define BALL_COMET_MS           500   
#endif
#ifndef BALL_COMET_FRAME_MS
#define BALL_COMET_FRAME_MS     15    
#endif
#ifndef STARTUP_WIPE_MS_PER_STEP
#define STARTUP_WIPE_MS_PER_STEP 5   
#endif
#ifndef PAUSE_IDLE_MS
#define PAUSE_IDLE_MS           300000 
#endif
#ifndef SCORE_INTERVAL
#define SCORE_INTERVAL          5     
#endif
#ifndef BALL_LOW_CONFIRM_US
#define BALL_LOW_CONFIRM_US     1000  
#endif
#ifndef BALL_REARM_MS
#define BALL_REARM_MS           300   
#endif
#ifndef SCOREMORE_BALL_PULSE_MS
#define SCOREMORE_BALL_PULSE_MS 150   
#endif
#ifndef BR_CLOSE_AFTER_SWEEPBACK_MS
#define BR_CLOSE_AFTER_SWEEPBACK_MS 5000 
#endif
#ifndef NO_CATCH_TIMEOUT_MS
#define NO_CATCH_TIMEOUT_MS     30000 
#endif
#ifndef RESUME_AFTER_DECKUP_MS
#define RESUME_AFTER_DECKUP_MS  2000  
#endif
#ifndef FLASH_ON_MS
#define FLASH_ON_MS             120   
#endif
#ifndef FLASH_OFF_MS
#define FLASH_OFF_MS            120   
#endif
#ifndef FLASH_COUNT
#define FLASH_COUNT             3     
#endif
#ifndef COMET_LEN
#define COMET_LEN               4     
#endif

// =====================================================
// MASTER_TEST-SPECIFIC SETTINGS
// =====================================================
#ifndef IR_FLAP_COUNT
#define IR_FLAP_COUNT       5     
#endif
#ifndef IR_FLAP_WINDOW_MS
#define IR_FLAP_WINDOW_MS   200   
#endif
#ifndef SCISSOR_CYCLE_MS
#define SCISSOR_CYCLE_MS    2000  
#endif
#ifndef WIPE_MS
#define WIPE_MS             500
#endif
#ifndef WIPE_FRAME_MS
#define WIPE_FRAME_MS       15
#endif
#ifndef RAINBOW_FRAME_MS
#define RAINBOW_FRAME_MS    20
#endif
#ifndef COMET_MS
#define COMET_MS            500   
#endif
#ifndef COMET_FRAME_MS
#define COMET_FRAME_MS      15    
#endif
#ifndef PDROP_SETTLE_MS
#define PDROP_SETTLE_MS         500   
#endif
#ifndef PDROP_RAISE_SETTLE_MS
#define PDROP_RAISE_SETTLE_MS   1300  
#endif
#ifndef PDROP_DROP_MS
#define PDROP_DROP_MS           800   
#endif
