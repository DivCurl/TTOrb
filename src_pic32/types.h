#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>

typedef char bool;

typedef struct {
    uint16_t r;
    uint16_t g;
    uint16_t b;
} rgb_t;

typedef struct {
    int tick;      // total ticks
    int pre;       // preset
    bool en;         // timer enabled
    bool dn;         // timer done
} timer_t;

// animations descriptor enums
enum {
    AN_SPIRAL_WALK = 0,
   // AN_METEOR,
    AN_RINGS,
    AN_RAINBOW_PULSE,
    AN_RAINBOW_SPIRAL,
    AN_SINUSOIDS,
    AN_TRACERS_HORZ,
    AN_TRACERS_VERT,
    AN_RAINBOW_RANDOM_FILL,
    // AN_RED_PULSE,
    // AN_GREEN_PULSE,
    // AN_BLUE_PULSE,
    // AN_CYAN_PULSE,
    // AN_MAGENTA_PULSE,
    // AN_RAINBOW_STROBE,
    AN_RAIN,
    //AN_SWAP,
    AN_METEOR,
    AN_RAINBOW_SOLID
};

enum {
   DRAW_OPT_HIRES,   //  allows for 'point' display during refresh (but also looks more 'stroby'
   DRAW_OPT_LORES    //  allows for smoother display during refresh but looks more 'wipy'
};

#endif