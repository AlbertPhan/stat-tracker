// color_utils.h
// Posted by Leszek Szary on stackoverflow.com
#include <stdint.h>

typedef struct RgbColor
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} RgbColor;

typedef struct HsvColor
{
    uint8_t h;
    uint8_t s;
    uint8_t v;
} HsvColor;


void HsvToRgb(const HsvColor *hsv, RgbColor *rgb);
