// Trip Toys "Orb" Prototype
// Design and programming by Mike Dillmann, 5/16/2016

#define _SUPPRESS_PLIB_WARNING

/* PIC32 Model PIC32MX250F128B */
#include <p32xxxx.h>    // pic32 library functions
#include <plib.h>       // peripheral library functions
#include <stdlib.h>
// #include <stdint.h>
#include <math.h>
#include "types.h"
#include "defs.h"

// Configuration Bits
#pragma config POSCMOD      = HS            // Primary oscillator using high speed crystal mode
#pragma config FNOSC        = PRIPLL        // Internal Fast RC oscillator (4 MHz) w/ PLL
#pragma config FPLLIDIV     = DIV_1         // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL      = MUL_20        // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV     = DIV_2         // Divide After PLL (now 40 MHz)
#pragma config FPBDIV       = DIV_1         // Divide core clock by 1 for peripheral bus (=20MHz Fpb)
#pragma config FWDTEN       = OFF           // Watchdog Timer Disabled
#pragma config ICESEL       = ICS_PGx1      // ICE/ICD Comm Channel Select
#pragma config JTAGEN       = OFF           // Disable JTAG
#pragma config FSOSCEN      = OFF           // Disable Secondary Oscillator

const uint8_t cmdData = 0x25;   // command 0x25 data
const uint8_t fcData = 0x12;    // function data
const unsigned char ascii[455] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x5f,0x5f,0x00,0x00,	//  !
    0x00,0x03,0x00,0x03,0x00,0x14,0x7f,0x14,0x7f,0x14,	// "#
    0x24,0x2a,0x7f,0x2a,0x12,0x23,0x13,0x08,0x64,0x62,	// $%
    0x36,0x49,0x55,0x22,0x50,0x00,0x05,0x03,0x00,0x00,	// &'
    0x00,0x1c,0x22,0x41,0x00,0x00,0x41,0x22,0x1c,0x00,	// ()
    0x14,0x08,0x3e,0x08,0x14,0x08,0x08,0x3e,0x08,0x08,	// *+
    0x00,0x50,0x30,0x00,0x00,0x08,0x08,0x08,0x08,0x08,	// ,-
    0x00,0x60,0x60,0x00,0x00,0x20,0x10,0x08,0x04,0x02,	// ./
    0x3e,0x51,0x49,0x45,0x3e,0x00,0x42,0x7f,0x40,0x00,	// 01
    0x42,0x61,0x51,0x49,0x46,0x21,0x41,0x45,0x4b,0x31,	// 23
    0x18,0x14,0x12,0x7f,0x10,0x27,0x45,0x45,0x45,0x39,	// 45
    0x3c,0x4a,0x49,0x49,0x30,0x01,0x71,0x09,0x05,0x03,	// 67
    0x36,0x49,0x49,0x49,0x36,0x06,0x49,0x49,0x29,0x1e,	// 89
    0x00,0x36,0x36,0x00,0x00,0x00,0x56,0x36,0x00,0x00,	// :;
    0x08,0x14,0x22,0x41,0x00,0x14,0x14,0x14,0x14,0x14,	// <=
    0x00,0x41,0x22,0x14,0x08,0x02,0x01,0x51,0x09,0x06,	// >?
    0x32,0x49,0x79,0x41,0x3e,0x7e,0x11,0x11,0x11,0x7e,	// @A
    0x7f,0x49,0x49,0x49,0x36,0x3e,0x41,0x41,0x41,0x22,	// BC
    0x7f,0x41,0x41,0x22,0x1c,0x7f,0x49,0x49,0x49,0x41,	// DE
    0x7f,0x09,0x09,0x09,0x01,0x3e,0x41,0x49,0x49,0x7a,	// FG
    0x7f,0x08,0x08,0x08,0x7f,0x00,0x41,0x7f,0x41,0x00,	// HI
    0x20,0x40,0x41,0x3f,0x01,0x7f,0x08,0x14,0x22,0x41,	// JK
    0x7f,0x40,0x40,0x40,0x40,0x7f,0x02,0x0c,0x02,0x7f,	// LM
    0x7f,0x04,0x08,0x10,0x7f,0x3e,0x41,0x41,0x41,0x3e,	// NO
    0x7f,0x09,0x09,0x09,0x06,0x3e,0x41,0x51,0x21,0x5e,	// PQ
    0x7f,0x09,0x19,0x29,0x46,0x46,0x49,0x49,0x49,0x31,	// RS
    0x01,0x01,0x7f,0x01,0x01,0x3f,0x40,0x40,0x40,0x3f,	// TU
    0x1f,0x20,0x40,0x20,0x1f,0x3f,0x40,0x38,0x40,0x3f,	// VW
    0x63,0x14,0x08,0x14,0x63,0x07,0x08,0x70,0x08,0x07,	// XY
    0x61,0x51,0x49,0x45,0x43,0x00,0x7f,0x41,0x41,0x00,	// Z[
    0x02,0x04,0x08,0x10,0x20,0x00,0x41,0x41,0x7f,0x00,	// \]
    0x04,0x02,0x01,0x02,0x04,0x40,0x40,0x40,0x40,0x40,	// ^_
    0x00,0x01,0x02,0x04,0x00,0x20,0x54,0x54,0x54,0x78,	// `a
    0x7f,0x48,0x44,0x44,0x38,0x38,0x44,0x44,0x44,0x20,	// bc
    0x38,0x44,0x44,0x48,0x7f,0x38,0x54,0x54,0x54,0x18,	// de
    0x08,0x7e,0x09,0x01,0x02,0x0c,0x52,0x52,0x52,0x3e,	// fg
    0x7f,0x08,0x04,0x04,0x78,0x00,0x44,0x7d,0x40,0x00,	// hi
    0x20,0x40,0x44,0x3d,0x00,0x7f,0x10,0x28,0x44,0x00,	// jk
    0x00,0x41,0x7f,0x40,0x00,0x7c,0x04,0x18,0x04,0x78,	// lm
    0x7c,0x08,0x04,0x04,0x78,0x38,0x44,0x44,0x44,0x38,	// no
    0x7c,0x14,0x14,0x14,0x08,0x08,0x14,0x14,0x18,0x7c,	// pq
    0x7c,0x08,0x04,0x04,0x08,0x48,0x54,0x54,0x54,0x20,	// rs
    0x04,0x3f,0x44,0x40,0x20,0x3c,0x40,0x40,0x20,0x7c,	// tu
    0x1c,0x20,0x40,0x20,0x1c,0x3c,0x40,0x30,0x40,0x3c,	// vw
    0x44,0x28,0x10,0x28,0x44,0x0c,0x50,0x50,0x50,0x3c,	// xy
    0x44,0x64,0x54,0x4c,0x44				// z
};
uint8_t bcData[ 3 ] = { GBRT_100_PERC, GBRT_100_PERC, GBRT_100_PERC };    // brightness control data - order by: B, G, R (init at max value: 127)
uint16_t gAnimList[ MAX_ANIM + 1 ] = { 0 };  
volatile rgb_t colorBuff[ MAX_THETA_DIV + X_OFFSET ][ MAX_PHI_DIV + Y_OFFSET ];
volatile rgb_t frameBuff[ MAX_THETA_DIV + X_OFFSET ][ MAX_PHI_DIV + Y_OFFSET ];
volatile int gRefreshAngle = 0;
volatile int gRefCount = 0;
volatile int gCount = 0;
volatile float gRGBAngle = 0;
volatile timer_t gTimer, gFadeTimer;
volatile int gAnimPtr = 0;      // initially set to zero for test animation
volatile int gModeFlags = 0;
volatile int gRefreshOpts;      // hires or lores
int it = 0; // current animation iteration
int dummy;

// Bit-shift a word of data out to TLC5971 driver (big endian)
void D_ShiftWord( uint16_t word, uint16_t mask ) {
    int i;
    for ( i = 15; i >= 0; i-- ) {
        // the mask is needed to avoid clocking bits that aren't aligned in the PWM driver control packet
        if ( mask >> i & 0x01 ) {
            if ( word >> i & 0x01 ) {
                LATBSET = SDTI;
            }
            LATBSET = SCKI;
            LATBCLR = SCKI | SDTI;
        }
    }
}

// Bit-shift a byte of data out to TLC5971 driver (big endian)
void D_ShiftByte( uint8_t byte, uint8_t mask ) {
    int i;
    for ( i = 7; i >= 0; i-- ) {
        // the mask is needed to avoid clocking bits that aren't aligned in the PWM driver control packet
        if ( mask >> i & 0x01 ) {
            if ( byte >> i & 0x01 ) {
                LATBSET = SDTI;
            }
            LATBSET = SCKI;
            LATBCLR = SCKI | SDTI;
        }
    }
}

// Bit-shift a byte of data out to TLC5971 driver (big endian)
void D_ShiftBits( uint32_t chunk, uint8_t numBits ) {
    int i;
    for ( i = ( numBits - 1 ); i >= 0; i-- ) {
        if ( chunk >> i & 0x01 ) {
            LATBSET = SDTI;
        }
        LATBSET = SCKI;
        LATBCLR = SCKI | SDTI;
    }
}

// blocking mS delay function
void D_msDelay ( unsigned int delay ) {
    // Timer startup
    if ( !gTimer.en ) {
        gTimer.tick = 0;
        gTimer.dn = 0;
        gTimer.en = 1;
    }

    // 1 tick every millisecond
    while ( gTimer.tick < delay ) {
        Nop();
    }

    gTimer.en = 0;
    gTimer.dn = 1;
}

float D_Remap( float in, float inMin, float inMax, float outMin, float outMax ) {
    float out;
    out = ( in - inMin ) / ( inMax - inMin ) * ( outMax - outMin ) + outMin;

    return out;
}

float D_Abs( float in ) {
    return ( in > 0 ? in : -in );
}

void D_LimitArray( float* in, float min, float max ) {
    if ( *in < min )  {
        *in = min;
    }
    if ( *in > max ) {
        *in = max;
    }
}

float D_Limit( float in, float min, float max ) {
    if ( ( min < in ) && ( max > in ) ) {
        return in;
    } else {
        return ( in < min ? min : max );
    }
}

int32_t D_Round( float in ) {
    int16_t s = in / D_Abs( in );
    return (int16_t)(s * ( D_Abs( in ) + 0.5 ) );
}

void D_RandArray1D( uint16_t* array, uint16_t arraySize, uint16_t passes ) {
    // fill  array with contiguous values to start with
    uint16_t rnd1, rnd2, tmp;
    for ( uint16_t i = 0; i < arraySize; i++ ) {
        array[ i ] = i;    // allows first element to start with non-zero value
    }

    for ( uint16_t i = 0; i <= passes; i++ ) {
        // swap element with random element
        rnd1 = rand() % arraySize;
        rnd2 = rand() % arraySize;
        tmp = array[ rnd1 ];
        array[ rnd1 ] = array[ rnd2 ];
        array[ rnd2 ] = tmp;
    }
}

void D_RandArray2D( int array1Size, int array2Size, uint16_t array[ ][ array2Size ], int passes ) {
    // fill  array1 with contiguous values to start with
    int rnd1, rnd2, rnd3, rnd4, tmp;
    for ( int j = 0; j < array2Size; j++ ) {
        for ( int i = 0; i < array1Size; i++ ) {
            array[ i ][ j ] = i;
        }
    }

    // 'randomize' 1st array
    for ( int k = 0; k <= passes; k++ ) {
        // swap element with random element
        rnd1 = rand() % array1Size; // random src x
        rnd2 = rand() % array1Size; // random dest x
        rnd3 = rand() % array2Size; // random src y
        rnd4 = rand() % array2Size; // random dest y

        tmp = array[ rnd1 ][ rnd3 ];
        array[ rnd1 ][ rnd3 ] = array[ rnd2 ][ rnd4 ];  // swap src & dest
        array[ rnd2 ][ rnd4 ] = tmp;
    }
}

static inline void D_UpdateFB() {
    memcpy ( frameBuff, colorBuff, sizeof( frameBuff ) );
}

// Refreshes the entire LED display by updating TLC5971 drivers
void D_Refresh( int blankMode ) {
    int i, j;
    // Because we need to retransmit the entire 224-bit packet
    // for each TLC5971 in cascade, and thus each 4th RGB LED, I've wrapped the
    // refresh logic up in a convenient loop that can be configured to
    // easily adjust for variable cascade configurations. Just #def PWM_DEVICE_MAX accordingly!
    for ( i = ( PWM_DEVICE_MAX - 1 ); i >= 0; i-- ) {    // need to shift MSB out first for proper sequencing
        // Send 25h cmd, control, and global brightness packet first
        D_ShiftBits( cmdData, 6 );       // command 25h - latch
        D_ShiftBits( fcData, 5 );         // optional config params
        D_ShiftBits( bcData[ 0 ], 7 );    // BC control - B
        D_ShiftBits( bcData[ 1 ], 7 );    // BC control - G
        D_ShiftBits( bcData[ 2 ], 7 );    // BC control - R
        // used to offset the color buffer in the loop below depending on which 5971 driver is currently being addressed.
        // always multiples of four (4) since each 5971 needs to process all LEDs in packet even if not connected
        int iMin = 4*i;
        int iMax = iMin + 4;
         for ( j = ( iMax - 1 ); j >= iMin; j-- ) {
             // blankmode == 0 -> update display via framebuffer
             if ( !blankMode ) {
                 D_ShiftBits( frameBuff[ gRefreshAngle ][ j ].b, 16 );
                 D_ShiftBits( frameBuff[ gRefreshAngle ][ j ].g, 16 );
                 D_ShiftBits( frameBuff[ gRefreshAngle ][ j ].r, 16 );

             // blankmode == 1 -> blank the display
             } else {
                 D_ShiftBits( 0, 16 );
                 D_ShiftBits( 0, 16 );
                 D_ShiftBits( 0, 16 );
             }
        }
    }

    // Short delay before next refresh to allow internal latch to fire
    // in the TLC5971 (dwell for period 8x last SCKI strobe per documentation)
    for( i = 0; i < 16; i++ ) {
        asm("nop");
    }
}

// Returns integer buffer address given polar input angle ( 0-360 deg ).
// Need to add bounds checking!!!
int D_Map360DegToArray( float deg ) {
    // output should vary between 0 and MAX_ANGLE_DIV
    // TEST CASE - FOR MAX_ANGLE_DIV = 12, '90 deg' INPUT should yield '3' OUTPUT
    // smallest divison = 360 / MAX_ANGLE_DIV
    return ( deg / ( 360.0f / MAX_THETA_DIV ) );
}

int D_Map180DegToArray ( float deg ) {
    return ( deg / ( 180.0f / MAX_PHI_DIV ) );
}

void D_ChangeT2Period( int period ) {
    CloseTimer2();
    OpenTimer2( T2_ON | T2_PS_1_64 | T2_SOURCE_INT, period );
    ConfigIntTimer2( T2_INT_ON | T2_INT_PRIOR_6 );
}

void D_ChangeT4Period( int period ) {
    CloseTimer4();
    OpenTimer4( T4_ON | T4_PS_1_64 | T4_SOURCE_INT, period );
    ConfigIntTimer4( T4_INT_ON | T4_INT_PRIOR_6 );
}


void D_GetChar ( char chr, unsigned char *dst ) {
    uint8_t i;
    // our bitmap font starts at ascii char 32 (decimal, space char). This ensures the input
    // ascii character is aligned with the start of the bitmap.
    chr -= 32;

    // loop through each byte of the array and store slice in dst[i] until entire char is built
    for ( i = 0; i < 5; i++ ) {
        dst[ i ] = ascii[ ( chr * 5 ) + i ] ;
    }
}


// Assigns the input RGB color to the specified voxel
void V_Set( int x, int y, rgb_t color, float brt ) {
    colorBuff[ x + X_OFFSET ][ y + Y_OFFSET ].r = color.r * brt / 100.0;
    colorBuff[ x + X_OFFSET ][ y + Y_OFFSET ].g = color.g * brt / 100.0;
    colorBuff[ x + X_OFFSET ][ y + Y_OFFSET ].b = color.b * brt / 100.0;
}

// Set surface voxel coordinate by degree angle
void V_SetSurfaceDeg( int theta, int phi, rgb_t color, float brt ) {
    if ( ( theta >= 0 ) && ( theta <= 360 ) && ( phi >= 0 ) && ( phi <= 180 ) ) {
        V_Set( D_Map360DegToArray( theta ), D_Map180DegToArray( phi ), color, brt );
    }
}

// Set surface voxel coordinate by radian angle
void V_SetSurfaceRad( float theta, float phi, rgb_t color, float brt ) {
    // while loop to handle possible multiples of 360
    while ( theta >= TWO_PI ) {
        theta = theta - TWO_PI;
    }

    while ( phi >= PI ) {
        phi = phi - PI;
    }

    V_SetSurfaceDeg( theta * 360.0f / TWO_PI, phi * 180.0f / PI, color, brt );
}

// for Polar version, y == r and x == polar angle
void V_Clear( int x, int y ) {
    colorBuff[ x + X_OFFSET ][ y + Y_OFFSET ].r = 0;
    colorBuff[ x + X_OFFSET ][ y + Y_OFFSET ].g = 0;
    colorBuff[ x + X_OFFSET ][ y + Y_OFFSET ].b = 0;
}

void V_ClearSurfaceDeg( float theta, float phi ) {
    if ( ( theta >= 0 ) && ( theta <= 360 ) && ( phi >= 0 ) && ( phi <= 180 ) ) {
        V_Clear( D_Map360DegToArray( theta ), D_Map180DegToArray( phi) );
    }
}

void V_ClearSurfaceRad( float theta, float phi ) {
    // to handle possible multiples of 360
    while ( theta >= TWO_PI ) {
        theta = theta - TWO_PI;
    }

    while ( phi >= PI ) {
        phi = phi - PI;
    }

    V_ClearSurfaceDeg( theta * 360.0f / TWO_PI, phi * 180.0f / PI );
}

// for Polar version, y == r and x == polar angle
void V_BlankBuffer() {
    for ( int i = 0; i < MAX_THETA_DIV; i++ ) {
        for ( int j = 0; j < MAX_PHI_DIV; j++ ) {
            V_Clear( i, j );
        }
    }
}

// for Polar version, y == phi and x == theta
void V_BlankRow( int row ) {
    for ( int i = 0; i < MAX_THETA_DIV; i++ ) {
        V_Clear( i, row );
    }
}

// for Polar version, y == r and x == polar angle
void V_FillRowSolid( int row, rgb_t color, float brt ) {
    for ( int i = 0; i < MAX_THETA_DIV; i++ ) {
         V_Set( i, row, color,  brt );
    }
}

// for Polar version, y == r and x == polar angle
void V_FillColSolid( int col, rgb_t color, float brt ) {
    for ( int j = 0; j < MAX_PHI_DIV; j++ ) {
         V_Set( col, j, color, brt );
    }
}

void V_Fill( rgb_t color, float brt ) {
    int i, j;
    for ( i = 0; i < MAX_THETA_DIV; i++ ) {
        for ( j = 0; j < MAX_PHI_DIV; j++ ) {
            V_Set( i, j, color, brt );
        }
    }
}

void V_FillRGB( int r, int g, int b, float brt ) {
    rgb_t color = { .r = r, .g = g, .b = b };
    int i, j;
    for ( i = 0; i < MAX_PHI_DIV; i++ ) {
        for ( j = 0; j < MAX_THETA_DIV; j++ ) {
            V_Set( j, i, color, brt );
        }
    }
}

// for Polar version, y == r and x == polar angle
void V_BlankCol( int col ) {
    for ( int j = 0; j < MAX_PHI_DIV; j++ ) {
        V_Clear( col, j );
    }
}

void V_CopCol ( int src, int dest ) {
    for ( int j = 0; j < ( MAX_PHI_DIV + Y_OFFSET ); j++ ) {
        colorBuff[ dest ][ j ] = colorBuff[ src ][ j ];
    }
}

void V_CopRow ( int src, int dest ) {
    for ( int i = 0; i < MAX_THETA_DIV; i++ ) {
        colorBuff[ i ][ dest ] = colorBuff[ i ][ src ];
    }
}

void V_Mov( int x1, int y1, int x2, int y2 ) {
    colorBuff[ x2 + X_OFFSET ][ y2 + Y_OFFSET ] = colorBuff[ x1 + X_OFFSET ][ y1 + Y_OFFSET];
    V_Clear( x1, y1 );
}

void V_Swap ( int x1, int y1, int x2, int y2 ) {
    rgb_t tmp1, tmp2;
    tmp1 = colorBuff[ x1 + X_OFFSET ][ y1 + Y_OFFSET ];
    tmp2 = colorBuff[ x2 + X_OFFSET ][ y2 + Y_OFFSET ];
    colorBuff[ x1 + X_OFFSET ][ y1 + Y_OFFSET ] = tmp2;
    colorBuff[ x2 + X_OFFSET ][ y2 + Y_OFFSET ] = tmp1;
}

// TODO - VERIFY BOUNDS CHECKING
void V_ShiftDown( ) {
    // Move rows down vertically by one level and clear top row
    // start at bottom row
    for ( int j = 0; j < MAX_PHI_DIV; j++ ) {
        for ( int i = 0; i < MAX_THETA_DIV; i++) {
            colorBuff[ i ][ j + Y_OFFSET ] =  colorBuff[ i ][ j + Y_OFFSET + 1 ];
        }
    }
    V_BlankRow( MAX_PHI_DIV - 1);
}

// TODO - VERIFY BOUNDS CHECKING
void V_ShiftUp( ) {
    // Move rows up vertically by one level and clear bottom row
    // start at top row
    for ( int j = ( MAX_PHI_DIV - 1 ); j >= 0; j-- ) {
        for ( int i = 0; i < MAX_THETA_DIV; i++) {
            colorBuff[ i ][ j + Y_OFFSET ] =  colorBuff[ i ][ (j + Y_OFFSET) - 1 ];
        }
    }
    V_BlankRow( 0 );
}

// TODO - VERIFY BOUNDS CHECKING
void V_ShiftRight( ) {
    // Move rows up vertically by one level and clear bottom row
    // start at top row
    for ( int j = 0; j <= MAX_PHI_DIV; j++ ) {
        for ( int i = ( MAX_THETA_DIV - 1 ); i > 0; i--) {
            colorBuff[ i ][ j + Y_OFFSET ] =  colorBuff[ i - 1 ][ j + Y_OFFSET ];
        }
    }
    V_BlankCol( 0 );
}

// takes 8-bit RGB input value and maps to 16-bit value
int R_Upsample( uint8_t in ) {
    // 16-bit = total of 65535 values, 8-bit total of 255 values
    // For even distribution, 65535 / 255 = 257
    // -> output = input * 257
    return ( int ) ( in * 257 );
}

// note: valid angle must be between 0 & 360 deg - no bounds checking
rgb_t R_GetColorByAngle( float angle ) {
    rgb_t color;
    // clamp input to limits
    // D_LimitArray ( &angle, 0, 359 );
    // ( angle < 360 ) ? angle : ( (int)angle %= 360 );
    //while ( angle >= 360 ) {
    //    angle - 360;
    // )
    angle = fmod( angle, 360 );
    // 0 deg = 255, 0 0
    // 60 deg = 255, 255, 0
    // 120 deg = 0, 255, 0
    // 180 deg = 0, 255, 255
    // 240 deg = 0, 0 255
    // 300 deg = 255, 0, 255
    // delta-phi = 60 degrees / delta-x = 255    

    // increase g value, hold red at max
    if ( ( angle >= 0 ) && ( angle < 60 ) ) {
        color.r = 65535;
        color.g = ( angle * 1092.25 );
        color.b = 0;
        return color;
    }

    // decrease red value, hold g at max
    if ( ( angle >= 60 ) && ( angle < 120 ) ) {
        color.r = ( 65535 - (angle-60) * 1092.25 );
        color.g = 65535;
        color.b = 0;
        return color;
    }

    // increase blue value, hold g at max
    if ( ( angle >= 120 ) && ( angle < 180 ) ) {
        color.r = 0;
        color.g = 65535;
        color.b = ( (angle-120) * 1092.25 );
        return color;
    }

    // decrease G value, hold B at max
    if ( ( angle >= 180 ) && ( angle < 240 ) ) {
        color.r = 0;
        color.g = ( 65535 - (angle-180) * 1092.25 );
        color.b = 65535;
        return color;
    }

    // increase R value, hold B at max
    if ( ( angle >= 240 ) && ( angle < 300 ) ) {
        color.r = ( (angle-240) * 1092.25 );
        color.g = 0;
        color.b = 65535;
        return color;
    }

    // decrease B value, hold R at max
    if ( ( angle >= 300 ) && ( angle < 360 ) ) {
        color.r = 65535;
        color.g = 0;
        color.b = ( 65535 - (angle-300) * 1092.25 );
        return color;
    }
}

// Fades out the entire buffer by the specified step factor
void R_FadeOut( int steps, int fadeMode ) {
    int brtStep, tmp;       // tmp needs to be int so we can detect underflow in if-test when using step method
    if ( fadeMode == FADE_STEP ) {
        brtStep = 65535 / steps;
    }
    
    for ( int x = 0; x <= MAX_THETA_DIV; x++ ) {
        for ( int y = 0; y <= MAX_PHI_DIV; y++ ) {
            if ( fadeMode == FADE_STEP ) {  // use step method
                tmp = colorBuff[ x + X_OFFSET ][ y + Y_OFFSET ].r;
                if ( ( tmp -= brtStep ) <= 0 ) {
                    colorBuff[ x + X_OFFSET ][ y + Y_OFFSET ].r = 0;
                } else {
                    colorBuff[ x + X_OFFSET ][ y + Y_OFFSET ].r -= brtStep;
                }

                tmp = colorBuff[ x + X_OFFSET ][ y + Y_OFFSET ].g;
                if ( ( tmp -= brtStep )  <= 0 ) {
                    colorBuff[ x + X_OFFSET ][ y + Y_OFFSET ].g = 0;
                } else {
                    colorBuff[ x + X_OFFSET ][ y + Y_OFFSET ].g -= brtStep;
                }

                tmp = colorBuff[ x + X_OFFSET ][ y + Y_OFFSET ].b;
                if ( ( tmp -= brtStep )  <= 0 ) {
                    colorBuff[ x + X_OFFSET ][ y + Y_OFFSET ].b = 0;
                } else {
                    colorBuff[ x + X_OFFSET ][ y + Y_OFFSET ].b -= brtStep;
                }
            } else {    // use shift method
                colorBuff[ x + X_OFFSET ][ y + Y_OFFSET ].r >>= steps;
                colorBuff[ x + X_OFFSET ][ y + Y_OFFSET ].g >>= steps;
                colorBuff[ x + X_OFFSET ][ y + Y_OFFSET ].b >>= steps;
            }
        }
    }
}

// Moves the color through the color wheel and returns the current value
rgb_t R_GetColorFade( int delay, float delta ) {
    if ( !gFadeTimer.en && ( delay > 0 ) ) {
        gFadeTimer.pre = delay;
        gFadeTimer.en = 1;
    }
    // Let the interrupt do its thing...or process and return angle immediately
    // if delay is zero to fade as quickly as possible
    if ( ( gFadeTimer.en && gFadeTimer.dn ) || ( delay == 0 ) ) {
        // wrap around
        if ( ( gRGBAngle += delta ) >= 360.0 ) {
            gRGBAngle -= 360.0;

        }
        gFadeTimer.dn = 0;  // reset the timer
    }
    
    return ( R_GetColorByAngle ( gRGBAngle ) );
}

// Strobe the display at a specified rate
void A_Strobe( int msDelay ) {
   bcData[0] = 0;
   bcData[1] = 0;
   bcData[2] = 0;
   D_msDelay( msDelay );
   bcData[0] = GBRT_100_PERC;
   bcData[1] = GBRT_100_PERC;
   bcData[2] = GBRT_100_PERC;
   D_msDelay( msDelay );
}

// Strobe the display at a specified rate
void A_StrobeByRow( int msDelay ) {
    static int row = 0;
    int i;
    if ( ++row > ( MAX_PHI_DIV - 1 ) ) {
        row = 0;
    }

    for ( i = 0; i < ( MAX_PHI_DIV - 1 ); i++ ) {
        if ( i != row ) {
            V_BlankRow( i );
        }
    }
    D_msDelay( msDelay );
}

void A_ScrollText( const char *str, rgb_t color ) {
    int x, y, i;
    unsigned char chr[5];   // stores the current ascii character

    // Loop through each character in the string until null byte is hit
    while ( *str ) {
        // Get the current character in the pointer and move it into chr array ( 5 bytes )
        D_GetChar( *str++, chr );
        int rndAngle = rand() % 360;
        // build the current character on the display
        for ( x = 0; x < 5; x++ ) {   // 5 bytes per char
            for ( y = 0; y < 8; y++ )	{
                if ( chr[ x ] & ( 0x80 >> y ) ) {
                    V_Set( x * 2, y, color, 100 );
                }
            }
        }

        D_UpdateFB();
        D_msDelay( 350 );

        // scroll it - shift 8x x-coords before drawing next char
        for ( i = 0; i < MAX_THETA_DIV; i++ ) {
            D_msDelay( 50 );
            V_ShiftRight();
            D_UpdateFB();
        }
    }   // end while loop

    // Shift out last character
    for ( i = 0; i <= MAX_THETA_DIV; i++ ) {
        D_msDelay( 50 );
        V_ShiftRight();
        D_UpdateFB();
    }
}

bool A_Swap( uint16_t* rndArray ) {
    static uint16_t i, j, rndCount, step = 0;
    if ( step == 0 ) {
        rgb_t clr1, clr2;
        uint16_t clrDelta = 360 / MAX_THETA_DIV;
        V_BlankBuffer();
        memset( rndArray, 0, sizeof( *rndArray ) * MAX_THETA_DIV );
        D_RandArray1D( rndArray, ( sizeof( *rndArray ) * MAX_THETA_DIV ) / sizeof( uint16_t ), ( MAX_THETA_DIV * 3 ) );
        
        dummy = rand() % 360;
        // clr1 = R_GetColorByAngle( 0 );
        // clr2 = R_GetColorByAngle( 180  );    // ensure colors are 180 degrees offset
        // TODO - change rings to faded colors for each voxel, but offset 
        // V_FillRowSolid( 0, clr1, 100 );
        // V_FillRowSolid( 9, clr2, 100 );
        for ( int ii = 0; ii < MAX_THETA_DIV; ii++ ) {
            V_Set( ii, 0, R_GetColorByAngle( dummy ), 100 );
            V_Set( ii, 9, R_GetColorByAngle( dummy + 180 ), 100 );
            dummy += clrDelta;
        }
        ( rndCount = rand() % 5 ) > 0 ? rndCount : ( rndCount = 1 );   // number of simultaneous swaps to start with
    } else {    // not first iteration
        if ( j < 9 ) {
            for ( int k = 0; k < rndCount; k++ )  {  // offset index by k-amount as we loop
                if ( j == 4 ) {
                    // V_Swap( i + k, j, i + k, j + 1 );
                    V_Swap( rndArray[i + k], j, rndArray[i + k], j + 1 );
                } else {
                    // V_Mov( i + k, j, i + k, j + 1 );            // move bottom voxel up one
                    // V_Mov( i + k, 9 - j, i + k, 9 - j - 1 );    // decrement top voxel down one
                    V_Mov( rndArray[i + k], j, rndArray[i + k], j + 1 );            // move bottom voxel up one
                    V_Mov( rndArray[i + k], 9 - j, rndArray[i + k], 9 - j - 1 );    // decrement top voxel down one
                }       
            }
            j++;
        } else {  // last j coord hit
            //if ( ( i +=  rndCount + 1 ) > MAX_THETA_DIV ) {          // update index
            //    i -= MAX_THETA_DIV;
            //}
            ( i += rndCount ) <= MAX_THETA_DIV ? i : ( i -= MAX_THETA_DIV ); // bounds check to ensure no theta-coords are skipped
            ( rndCount = rand() % 5 ) > 0 ? rndCount : ( rndCount = 1 );  // update random voxel count on next swap round - ensures we move fwd at least one position
            j = 0;                  // reset start y-coords
            // i++;
        }
    }

    if ( i >= MAX_THETA_DIV ) { // done
        step = i = j = 0;         // reset
        return ( 1 );
    } else {
        step++;       // keep going
        return ( 0 );
    }
}

bool A_Pulse( rgb_t clr, int brtDelta ) {
    static int step = 1;
    static int dwell = 0;
    bool cycleDone = 0;
    static float brt = 100;

    V_Fill( clr, brt );

    switch ( step ) {
        // gradually decreasing brightness
        case 1:
            if ( ( brt -= brtDelta ) <= 0 ) {
                brt = 0;
                step = 2;
            }
            break;

        // at this point the display is completely dimmed.
        // dwell here for a few steps...looks more pleasant this way
        case 2:
            if ( ++dwell == 5 ) {
                dwell = 0;
                step = 3;
            }
            break;

        // gradually increasing brightness
        case 3:
            if ( ( brt += brtDelta ) >= 100 ) {
                brt = 100;
                step = 4;
            }
            break;

        // at this point the display is completely bright.
        // dwell here for a few steps...
        case 4:
            if ( ++dwell == 5 ) {
                dwell = 0;
                step = 1;   // reset back to step 1 and repeat
                cycleDone = 1;
            }            
            break;
    
        default:
            break;
    } // end switch

    return ( cycleDone );
}

void A_SpiralWalk() {
    // Draw first leg of spiral
    V_SetSurfaceDeg( 0, 0, R_GetColorFade( 25, 5 ), 100 );
    V_SetSurfaceDeg( 18, 18, R_GetColorFade( 25, 5 ), 100 );
    V_SetSurfaceDeg( 36, 36, R_GetColorFade( 25, 5 ), 100 );
    V_SetSurfaceDeg( 54, 54, R_GetColorFade( 25, 5 ), 100 );
    V_SetSurfaceDeg( 72, 72, R_GetColorFade( 25, 5 ), 100 );
    V_SetSurfaceDeg( 90, 90, R_GetColorFade( 25, 5 ), 100 );
    V_SetSurfaceDeg( 108, 108, R_GetColorFade( 25, 5 ), 100 );
    V_SetSurfaceDeg( 126, 126, R_GetColorFade( 25, 5 ), 100 );
    V_SetSurfaceDeg( 144, 144, R_GetColorFade( 25, 5 ), 100 );
    V_SetSurfaceDeg( 162, 162, R_GetColorFade( 25, 5 ), 100 );

    // Draw second leg of spiral, with colors inverted
    V_SetSurfaceDeg( 180, 0, R_GetColorFade( 25, 5 ), 100 );
    V_SetSurfaceDeg( 198, 18, R_GetColorFade( 25, 5 ), 100 );
    V_SetSurfaceDeg( 216, 36, R_GetColorFade( 25, 5 ), 100 );
    V_SetSurfaceDeg( 234, 54, R_GetColorFade( 25, 5 ), 100 );
    V_SetSurfaceDeg( 252, 72, R_GetColorFade( 25, 5 ), 100 );
    V_SetSurfaceDeg( 270, 90, R_GetColorFade( 25, 5 ), 100 );
    V_SetSurfaceDeg( 288, 108, R_GetColorFade( 25, 5 ), 100 );
    V_SetSurfaceDeg( 306, 126, R_GetColorFade( 25, 5 ), 100 );
    V_SetSurfaceDeg( 324, 144, R_GetColorFade( 25, 5 ), 100 );
    V_SetSurfaceDeg( 342, 162, R_GetColorFade( 25, 5 ), 100 );
}

void A_Rain() {
    int i, r_i, rnd;
    dummy = MAX_THETA_DIV * 0.05f;  // we can play with the factor a little. 0.05 looks pretty good and doesn't clutter the display too badly
    rnd = rand() % dummy;           // max random number of 'droplets' to start with on top row every cycle
    V_ShiftDown();
    for ( i = 0; i <= rnd; i++ ) {
        r_i = rand() % ( MAX_THETA_DIV - 1);   // random theta-coord voxels
        dummy = rand() % 360;    // random color
        // V_Set( r_i, ( MAX_PHI_DIV - 1 ), R_GetColorFade( 10, 15 ), 100 );    // rainbow drops
        V_Set( r_i, ( MAX_PHI_DIV - 1 ), R_GetColorFade( 25, 5 ), 100 );        
    }
}

void A_TracersVert() {
    static int step1 = 0;
    static rgb_t clr1, clr2;

    // Ugly as fuck; testing proof of concept
    switch ( step1 ) {
        case 0:
            V_BlankBuffer();
            clr1 = R_GetColorByAngle( ( rand() % 360 ) );
            clr2 = R_GetColorByAngle( ( rand() % 180 ) );
            V_Set( 0, 0, clr1, 100);

            // Mirror
            V_Set( 0, 9, clr1, 100);
            break;
        case 1:
            V_Set( 0, 1, clr1, 100 );
            V_Set( 0, 0, clr1, 70 );

            //Mirror
            V_Set( 0, 8, clr1, 100 );
            V_Set( 0, 9, clr1, 70 );
            break;
        case 2:
            V_Set( 0, 2, clr1, 100 );
            V_Set( 0, 1, clr1, 70 );
            V_Set( 0, 0, clr1, 40 );

            //Mirror
            V_Set( 0, 7, clr1, 100 );
            V_Set( 0, 8, clr1, 70 );
            V_Set( 0, 9, clr1, 40 );
            break;
        case 3:
            V_Set( 0, 3, clr1, 100 );
            V_Set( 0, 2, clr1, 70 );
            V_Set( 0, 1, clr1, 40 );
            V_Set( 0, 0, clr1, 10 );

            //mirror
            V_Set( 0, 6, clr1, 100 );
            V_Set( 0, 7, clr1, 70 );
            V_Set( 0, 8, clr1, 40 );
            V_Set( 0, 9, clr1, 10 );
            break;
        case 4:
            V_Set( 0, 4, clr1, 100 );
            V_Set( 0, 3, clr1, 70 );
            V_Set( 0, 2, clr1, 40 );
            V_Set( 0, 1, clr1, 10 );
            V_Set( 0, 0, clr1, 0 );

            //mirror
            V_Set( 0, 5, clr1, 100 );
            V_Set( 0, 6, clr1, 70 );
            V_Set( 0, 7, clr1, 40 );
            V_Set( 0, 8, clr1, 10 );
            V_Set( 0, 9, clr1, 0 );
            break;
        // Switch directions and start new color in vertical-center voxel
        case 5:
            V_Set( 0, 4, clr2, 100 );
            V_Set( 0, 3, clr1, 70 );
            V_Set( 0, 2, clr1, 40 );
            V_Set( 0, 1, clr1, 10 );
            V_Set( 0, 0, clr1, 0 );

            // mirror
            V_Set( 0, 5, clr2, 100 );
            V_Set( 0, 6, clr1, 70 );
            V_Set( 0, 7, clr1, 40 );
            V_Set( 0, 8, clr1, 10 );
            V_Set( 0, 9, clr1, 0 );
            break;
        case 6:
            V_Set( 0, 4, clr2, 70 );
            V_Set( 0, 3, clr2, 100 );
            V_Set( 0, 2, clr1, 0 );
            V_Set( 0, 1, clr1, 0 );
            V_Set( 0, 0, clr1, 0 );

            //mirror
            V_Set( 0, 5, clr2, 70 );
            V_Set( 0, 6, clr2, 100 );
            V_Set( 0, 7, clr1, 0 );
            V_Set( 0, 8, clr1, 0 );
            V_Set( 0, 9, clr1, 0 );
            break;
        case 7:
            V_Set( 0, 4, clr2, 40 );
            V_Set( 0, 3, clr2, 70 );
            V_Set( 0, 2, clr2, 100 );
            V_Set( 0, 1, clr1, 0 );
            V_Set( 0, 0, clr1, 0 );

            //mirror
            V_Set( 0, 5, clr2, 40 );
            V_Set( 0, 6, clr2, 70 );
            V_Set( 0, 7, clr2, 100 );
            V_Set( 0, 8, clr1, 0 );
            V_Set( 0, 9, clr1, 0 );
            break;
        case 8:
            V_Set( 0, 4, clr2, 10 );
            V_Set( 0, 3, clr2, 40 );
            V_Set( 0, 2, clr2, 70 );
            V_Set( 0, 1, clr2, 100 );
            V_Set( 0, 0, clr1, 0 );

            //mirror
            V_Set( 0, 5, clr2, 10 );
            V_Set( 0, 6, clr2, 40 );
            V_Set( 0, 7, clr2, 70 );
            V_Set( 0, 8, clr2, 100 );
            V_Set( 0, 9, clr1, 0 );
            break;
        case 9:
            V_Set( 0, 4, clr2, 0 );
            V_Set( 0, 3, clr2, 0 );
            V_Set( 0, 2, clr2, 40 );
            V_Set( 0, 1, clr2, 70 );
            V_Set( 0, 0, clr1, 100 );

            //mirror
            V_Set( 0, 5, clr2, 0 );
            V_Set( 0, 6, clr2, 0 );
            V_Set( 0, 7, clr2, 40 );
            V_Set( 0, 8, clr2, 70 );
            V_Set( 0, 9, clr1, 100 );
            break;

        default:
            break;
    }

    for ( int  i = 1; i < 6; i++ ) {
        int  dummy = ( i * D_Round( ( MAX_THETA_DIV / 6 ) ) );
        V_CopCol( 0, i * D_Round( ( MAX_THETA_DIV / 6 ) ) );
    }
    
    if ( ++step1 > 9 ) {
        step1 = 0;
    }
}

void A_TracersHorz() {
    static bool init, y = 0;
    static int xCoords[ 10 ] = { 0 };
    // on first scan, randomize starting x-coordinates
    if ( !init ) {
        for ( int i = 0; i <= 9; i++ ) {
            xCoords[ i ] = rand() % MAX_THETA_DIV;
        }
        init = 1;
    }

    R_FadeOut( 3, FADE_SHIFT );  // trails
    
    for ( int i = 0; i <= 9; i++ ) {
        V_Set( xCoords[ i ], i, R_GetColorByAngle( ( rand() % 360 ) ) , 100 );
        if ( i % 2 == 0 ) { // even coordinates increase CCW
            if ( ( xCoords[ i ] += 1 ) == MAX_THETA_DIV ) {
                xCoords[ i ] = 0;
            }
        } else {            // odd coordinates decrease CCW
            if ( ( xCoords[ i ] -= 1 ) == 0 ) {
                xCoords[ i ] = ( MAX_THETA_DIV - 1 );
            }
        }
    }
}

void A_Meteor() {
    static int x, y = 0;
    R_FadeOut( 3, FADE_SHIFT );
    V_Set( x, y, R_GetColorFade( 15, 5 ), 100 );
    if ( (x += 4) == ( 32 ) ) {    // every 1/2 cycle, move y-coord up one
        if ( ++y > MAX_PHI_DIV ) {
            y = 0;
        }
   } else {
        if ( x == MAX_THETA_DIV ) {
            x = 0;
        }
   }
}

bool A_Sinusoids( ) {
    static float phi = 0;
    static int i, angle = 0;
    int y;

    if ( i == 0 ) { // First pass - get new color
        angle = rand() % 360;
    }
    if ( ( i % 2 ) == 0 ) { // only plot even points
        R_FadeOut( 10, FADE_STEP );
        y = D_Round( ( 4 * sin ( phi ) + 4 ) );
        V_Set( i, y, R_GetColorByAngle( angle ), 100 );
        y = D_Round( ( 4 * sin ( phi + PI ) + 4 ) );
        V_Set( i, y, R_GetColorByAngle( angle + 180 ), 100 );
        // angle = ( angle + 3) % 360;
    }
    phi = fmod( ( phi + 0.19635 ), TWO_PI );    // phase ensures two full cycles over 64-step display
    
    if ( ++i == 64 ) {
        i = 0;
       // V_BlankBuffer();
        return ( 1 );
    } else {   
        return ( 0 );
    }

}

bool A_RandomRainbowFill( uint16_t* rndArray ) {
    // static uint16_t testArray[ MAX_THETA_DIV * MAX_PHI_DIV ];
    static int count, init = 0;
    int x, y;
    if ( init == 0 ) {
        memset( rndArray, 0, sizeof( *rndArray ) * MAX_THETA_DIV * MAX_PHI_DIV );
        D_RandArray1D( rndArray, sizeof( *rndArray ) * MAX_THETA_DIV * MAX_PHI_DIV / sizeof( uint16_t ), ( MAX_THETA_DIV * 6 ) ); // buffer with randomized theta (x) elements
        init = 1;
    }

    // loop through entire randomized buffer and set corresponding voxels on display
    if ( count < ( MAX_THETA_DIV * MAX_PHI_DIV ) ) {
        x = rndArray[ count ] % 64;
        y = rndArray[ count ] / 64;
        V_Set( x, y, R_GetColorByAngle( ( rand() % 360 ) ), 100);
        count++;
        return 0;
    } else {    // done
        count = init = 0;
        return 1;
    }
}

bool A_RainbowSpiral() {

    static uint16_t i, j = 0;
 
    V_Set( i, j, R_GetColorFade( 0, 3 ), 100 );

    if ( i++ >= MAX_THETA_DIV ) {
        i = 0;
        if ( j++ >= MAX_PHI_DIV ) {
            j = 0;
            return ( 1 );   // entire orb is lit
        }        
    }

    return ( 0 );
}

bool A_Rings() {
    static uint16_t i, j = 0;


    V_Set( i, 0, R_GetColorByAngle(120), 100 );
    V_Set( (MAX_THETA_DIV-i), 2, R_GetColorByAngle(230), 100 );
    V_Set( i, 4, R_GetColorByAngle(0), 100 );
    V_Set( (MAX_THETA_DIV-i), 6, R_GetColorByAngle(60), 100 );
    V_Set( i, 8, R_GetColorByAngle(300), 100 );


    if ( i++ >= MAX_THETA_DIV ) {
        i = 0;
        if ( (j += 2) > 8 ) {
            j = 0;
            return ( 1 );   // entire orb is lit
        }        
    }

    return ( 0 );
}

// FUCKED!
void A_Fire() {


}
// Animation switching function
// Not really necessary for ORB since no I/O but kept for consistency
int A_SwitchAnim( int its, int modeFlags ) {

    if ( TESTFLAG ( gModeFlags, MODE_FWD ) ) {
        CLRFLAG( gModeFlags, MODE_FWD | MODE_PREV );
        if ( ++gAnimPtr > MAX_ANIM ) {
            gAnimPtr = MIN_ANIM;
        }
        it = 0;
        return 1;
    }
    // go to previous animation and restore previous mode
    if ( TESTFLAG ( gModeFlags, MODE_PREV ) ) {
        CLRFLAG( gModeFlags, MODE_PREV | MODE_PREV );
        if ( --gAnimPtr < MIN_ANIM ) {
            gAnimPtr = MAX_ANIM;
        }
        it = 0;
        return 1;
    }

    // do all iterations in loop mode
    // ** Fall through when all iterations completed in loop mode
    if ( TESTFLAG( gModeFlags, MODE_LOOP ) ) {      
        while ( it++ < its ) {
            return 0;   // continue current animation
        }

    } else {    // in 'dwell' mode        
        it = 0;         // reset current iteration each scan to be safe
        return 0;       // continue current animation
    }

    // move on to next animation in sequence if ANIM_SWITCH_BYPASS flag is not set
    it = 0;
    
    if ( TESTFLAG( modeFlags, ANIM_SWITCH | ANIM_SKIP ) ) {
        if ( ++gAnimPtr > MAX_ANIM ) {
            // randomize animation list and reset pointer after all animations have been processed
            D_RandArray1D( gAnimList, sizeof(gAnimList) / sizeof( uint16_t ), ( MAX_ANIM * 2 ) );
            gAnimPtr = MIN_ANIM;
        }
    }

    return 1;
}

void A_Animate( ) {
    static bool firstPass = 1;
    bcData[0] = bcData[1] = bcData[2] = GBRT_100_PERC;

 // #define __TEST__
#ifdef __TEST__
    while ( 1 ) {
       while ( !A_RainbowSpiral( ) ) {
                    D_UpdateFB();
                    D_msDelay( 50 );
                }
    }

//    Nop();
#endif
    if ( firstPass ) {
        D_msDelay(4000);
        firstPass = 0;
        
    }

    // Clear display before starting new animation
    V_BlankBuffer();

    switch ( gAnimList [ gAnimPtr] ) {
        // Need to finish
        case AN_TRACERS_VERT: {
            gRefreshOpts = DRAW_OPT_HIRES;
            D_ChangeT2Period( TMR2_PERIOD - 10 );
            while ( !A_SwitchAnim( 256, ANIM_SWITCH ) ) {
                A_TracersVert();
                D_UpdateFB();
                D_msDelay( 125 );
            }
            D_ChangeT2Period( TMR2_PERIOD );
            break;
        }

        case AN_RAINBOW_RANDOM_FILL: {
            uint16_t* rndArray = malloc( sizeof(uint16_t) * MAX_THETA_DIV * MAX_PHI_DIV );
            gRefreshOpts = DRAW_OPT_HIRES;
            while ( !A_SwitchAnim( 1, ANIM_SWITCH ) ) {
                while ( !A_RandomRainbowFill( rndArray ) ) {
                    D_UpdateFB();
                    D_msDelay( 75 );
                }
                // Done - clear display
                V_BlankBuffer();
                D_UpdateFB();
                D_msDelay( 2000 );

                // Now do again but with fadeout
                while ( !A_RandomRainbowFill( rndArray ) ) {
                    D_UpdateFB();                    
                    D_msDelay( 75 );
                    R_FadeOut( 32, FADE_STEP );
                    D_UpdateFB();
                }
            }
            free( rndArray );
            break;
        }
        
        case AN_TRACERS_HORZ: {
            gRefreshOpts = DRAW_OPT_HIRES;
            while ( !A_SwitchAnim( 256, ANIM_SWITCH ) ) {
                D_UpdateFB();
                A_TracersHorz();
                D_msDelay( 100 );
            }
            break;
        }

        // Works but needs some cleanup
        case AN_SPIRAL_WALK: {
            gRefreshOpts = DRAW_OPT_HIRES;
            // fun with aliasing side effects...
            D_ChangeT2Period( TMR2_PERIOD - 20 );
            while ( !A_SwitchAnim( 256, ANIM_SWITCH ) ) {
                A_SpiralWalk();
                D_UpdateFB();
                D_msDelay( 75 );
            }
            D_ChangeT2Period( TMR2_PERIOD );
            break;
        }
        
        case AN_RAIN: {
            gRefreshOpts = DRAW_OPT_HIRES;
            while ( !A_SwitchAnim( 256, ANIM_SWITCH ) ) {
                A_Rain();
                D_UpdateFB();
                D_msDelay( 100 );
            }
            break;
        }

        case AN_METEOR: {
            gRefreshOpts = DRAW_OPT_HIRES;
            while ( !A_SwitchAnim( 256, ANIM_SWITCH ) ) {
                A_Meteor();
                D_UpdateFB();
                D_msDelay( 75 );
            }
            break;
        }

        /*
        case AN_RED_PULSE:      // boring            
            gRefreshOpts = DRAW_OPT_LORES;
            while ( !A_SwitchAnim( 1, ANIM_SWITCH ) ) {
                while ( !A_Pulse( R_GetColorByAngle( RGB_RED ), 1 ) ) {
                    D_UpdateFB();
                    D_msDelay( 100 );
                }
            }
            
            break;
         */

        /*

        case AN_GREEN_PULSE: {   // boring
            gRefreshOpts = DRAW_OPT_LORES;
            while ( !A_SwitchAnim( 1, ANIM_SWITCH ) ) {
                while ( !A_Pulse( R_GetColorByAngle( RGB_GREEN ), 1 ) ) {
                    D_UpdateFB();
                    D_msDelay( 100 );
                }
            }
            break;
        }
         */

        /*

        case AN_BLUE_PULSE: {   // boring            
            gRefreshOpts = DRAW_OPT_LORES;
            while ( !A_SwitchAnim( 1, ANIM_SWITCH ) ) {
                while ( !A_Pulse( R_GetColorByAngle( RGB_BLUE ), 1 ) ) {
                    D_UpdateFB();
                    D_msDelay( 100 );
                }
            }            
            break;
        }
         */

        /*

        case AN_MAGENTA_PULSE: { // boring
            gRefreshOpts = DRAW_OPT_LORES;
            while ( !A_SwitchAnim( 1, ANIM_SWITCH ) ) {
                while ( !A_Pulse( R_GetColorByAngle( RGB_MAGENTA ), 1 ) ) {
                    D_UpdateFB();
                    D_msDelay( 100 );
                }
            }
            break;
        }
         */

        /*

        case AN_CYAN_PULSE: { // boring
            gRefreshOpts = DRAW_OPT_LORES;
            while ( !A_SwitchAnim( 2, ANIM_SWITCH ) ) {
                while ( !A_Pulse( R_GetColorByAngle( RGB_CYAN ), 2 ) ) {
                    D_UpdateFB();
                    D_msDelay( 100 );
                }
            }
            break;
        }
         */

        case AN_RAINBOW_PULSE: {
            gRefreshOpts = DRAW_OPT_LORES;
            while ( !A_SwitchAnim( 1, ANIM_SWITCH ) ) {
                while ( !A_Pulse( R_GetColorFade( 0, 5 ), 2 ) ) {
                    D_UpdateFB();
                    D_msDelay( 50 );
                }
            }
            break;
        }
       
        case AN_SINUSOIDS:
            gRefreshOpts = DRAW_OPT_HIRES;
            while ( !A_SwitchAnim( 10, ANIM_SWITCH ) ) {
                while ( !A_Sinusoids() ) {
                    D_UpdateFB();
                    D_msDelay( 75 );
                }
            }
            break;
        

            /*
        case AN_SWAP: {
            uint16_t* rndArray = malloc( sizeof(uint16_t) * MAX_THETA_DIV );
            gRefreshOpts = DRAW_OPT_HIRES;
            while ( !A_SwitchAnim( 1, ANIM_SWITCH ) ) {
                while ( !A_Swap( rndArray ) ) {
                    D_UpdateFB();
                    D_msDelay( 125 );
                }
            }
            free( rndArray );
            break;
        }
             */

        case AN_RAINBOW_SOLID: {
            gRefreshOpts = DRAW_OPT_LORES;
            while ( !A_SwitchAnim( 256, ANIM_SWITCH ) ) {
                V_Fill( R_GetColorFade( 0, 5 ), 100 );
                D_UpdateFB();
                D_msDelay( 25 );
            }
            break;
        }


        case AN_RAINBOW_SPIRAL: {
            gRefreshOpts = DRAW_OPT_LORES;
            while ( !A_SwitchAnim( 3, ANIM_SWITCH ) ) {
                V_BlankBuffer();
                D_UpdateFB();
                while ( !A_RainbowSpiral( ) ) {
                    D_UpdateFB();
                    D_msDelay( 50 );
                }
            }
            break;
        }

        case AN_RINGS: {
            gRefreshOpts = DRAW_OPT_LORES;
            while ( !A_SwitchAnim( 3, ANIM_SWITCH ) ) {
                V_BlankBuffer();
                D_UpdateFB();
                while ( !A_Rings( ) ) {
                    D_UpdateFB();
                    D_msDelay( 50 );
                }
            }
            break;
        }
         

                                    
        default:
            A_SwitchAnim( 0, ANIM_SWITCH ); // ensures any un-used animations are skipped
            break;        
    } // end switch    
}

// Timer 4 interrupt handler for accurate timekeeping 
void __ISR ( _TIMER_4_VECTOR, /*ipl3*/ IPL3AUTO ) TMR4IntHandler( void ) {
    if ( gTimer.en ) {
        gTimer.tick++;
    } else {
        gTimer.dn = 0;
        gTimer.tick = 0;
    }

    if ( gFadeTimer.en ) {
        if ( !gFadeTimer.dn ) {
            if ( gFadeTimer.tick < gFadeTimer.pre ) {
                gFadeTimer.tick++;
            } else {
                gFadeTimer.dn = 1;
                gFadeTimer.tick = 0;
            }
        }
    } else {    // if timer is not enabled reset all
        gFadeTimer.dn = 0;
        gFadeTimer.tick = 0;
    }

    mT4ClearIntFlag();
}

// Timer 2 interrupt handler for refresh mux 
void __ISR ( _TIMER_2_VECTOR, /*ipl6*/ IPL6AUTO ) TMR2IntHandler( void ) {
    // Pulse display for 1 cycle
    // asm("di");
    if ( ( gRefCount < 1 ) || ( gRefreshOpts == DRAW_OPT_LORES ) ) {
        D_Refresh( 0 );
        if ( ++gRefreshAngle > ( MAX_THETA_DIV - 1 ) ) {
            gRefreshAngle = 0;
        }
    } else {    // blank display for 1 cycles to display voxels more sharply (less 'wipe' effect)
        D_Refresh( 1 );
    }

    if ( ++gRefCount > 1 ){
        gRefCount = 0;
    }
    
    mT2ClearIntFlag();
    // asm("ei");
}

int main() {
    SYSTEMConfig( SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE );
    mJTAGPortEnable( DEBUG_JTAGPORT_OFF );
    ANSELA = 0;
    ANSELB = 0;
    // Set PORTB tristate pin modes
    TRISBbits.TRISB2 = 0;       // PWM driver SCKI pin
    TRISBbits.TRISB3 = 0;       // PWM driver STDI pin

    OpenTimer2( T2_ON | T2_PS_1_64 | T2_SOURCE_INT, TMR2_PERIOD );
    ConfigIntTimer2( T2_INT_ON | T2_INT_PRIOR_6 );

    OpenTimer4( T4_ON | T4_PS_1_8 | T4_SOURCE_INT, TMR4_PERIOD );
    ConfigIntTimer4( T4_INT_ON | T4_INT_PRIOR_3 );
  
    INTEnableSystemMultiVectoredInt();
    srand( ReadCoreTimer() );
    SETFLAG( gModeFlags, MODE_LOOP );   // Orb currently only using LOOP mode

    // Start with randomized animation sequence
    D_RandArray1D( gAnimList, sizeof( gAnimList ) / sizeof( uint16_t ), 64 );
   // for ( int i = 0; i <= 10; i++ ) {
   //     gAnimList[ i ] = i;
   // }
    while ( 1 ) {
        A_Animate();
    }
    
    return ( EXIT_SUCCESS );
}