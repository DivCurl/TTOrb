// core clock and PIC32 setup stuff
#define SYS_FREQ            ( 40000000L )
#define SCKI                0x08        // clock pin = RB2 (pin 6)
#define SDTI                0x04        // data pin = RB3  (pin 7)
#define PWM_DEVICE_MAX      3           // total number of TLC5971 drivers

// geometry & timing
#define PI                  3.1415926535897932384626433832795f
#define PI_OVER_180         ( PI / 180.0f )
#define TWO_PI              ( PI * 2.0f )
#define MAX_R_DIV           1           // number of radial divisions in grid (held at 1 in this revision since only drawing outer surface of sphere)
#define MAX_THETA_DIV       64         // number of divisions in grid over x (theta) plane
#define MAX_PHI_DIV         10          // 10 LEDs -> number of divisions in grid over y (phi) plane
#define X_OFFSET            0
#define Y_OFFSET            2           // 2 LEDs in the light string are non-functional so offset them
#define MOTOR_RPM           495         // calibrated with motor at "high" speed
#define MOTOR_HZ            ( MOTOR_RPM / 60.0 )
#define REFRESH_HZ          ( MOTOR_HZ * MAX_THETA_DIV )
#define TMR2_PERIOD         ( ( ( ( 1.0 / REFRESH_HZ ) * SYS_FREQ / 64.0 ) - 1.0 ) / 2.0 )   // divisor of 2.0 to allow for display blanking every 1/2 division
#define TMR4_PERIOD         5000        // General timer refresh period -> yields 1khz = 1 tick every .001 sec
#define MIN_ANIM            0
#define MAX_ANIM            10
#define GBRT_100_PERC       127
#define GBRT_90_PERC        114
#define GBRT_80_PERC        102
#define GBRT_70_PERC        89
#define GBRT_60_PER         76
#define GBRT_50_PERC        63
#define GBRT_40_PERC        51
#define GBRT_30_PERC        38
#define GBRT_20_PERC        25
#define GBRT_10_PERC        13
#define GBRT_0_PERC         0

// some common RGB colors, by angle value
#define RGB_RED             0
#define RGB_ORANGE          30
#define RGB_YELLOW          60
#define RGB_SPRING_GREEN    90
#define RGB_GREEN           120
#define RGB_TURQOISE        150
#define RGB_CYAN            180
#define RGB_OCEAN           210
#define RGB_BLUE            240
#define RGB_VIOLET          270
#define RGB_MAGENTA         300
#define RGB_RASPBERRY       330
#define RGB_WHITE           -1
#define MODE_FWD            0x01
#define MODE_PREV           0x02
#define MODE_LOOP           0x10    // 1xh = loop, 0xh = dwell
#define FADE_IN             0x00
#define FADE_OUT            0x01
#define ANIM_SWITCH         0x01
#define ANIM_SKIP           0x02
#define COORD_ANGLE         0x00
#define COORD_XY            0x01
#define FADE_STEP           0x00
#define FADE_SHIFT          0x01

// some evil dirty macros
#define SETFLAG( a, b )     ( ( a ) |= ( b ) )
#define CLRFLAG( a, b )     ( ( a ) &= ~( b ) )
#define TESTFLAG( a, b )    ( ( a ) & ( b ) )