// color_utils.c
// Posted by Leszek Szary on https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both/14733008#14733008
// Modified to work with CH552


#include <color_utils.h>
#include <ch554.h>

void HsvToRgb(const HsvColor *hsv, RgbColor *rgb)
{
    __xdata uint8_t region; 
    __xdata uint16_t remainder, p, q, t;
    if (hsv->s == 0)
    {
        rgb->r = hsv->v;
        rgb->g = hsv->v;
        rgb->b = hsv->v;
        return;
    }
    else if(hsv->v == 0)
    {
        rgb->r = 0;
        rgb->g = 0;
        rgb->b = 0;
        return;
    }

    region = hsv->h / 43;
    remainder = (hsv->h - (region * 43)) * 6; 

    p = (hsv->v * (255 - hsv->s)) >> 8;
    q = (hsv->v * (255 - ((hsv->s * remainder) >> 8))) >> 8;
    t = (hsv->v * (255 - ((hsv->s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
        case 0:
            rgb->r = hsv->v; rgb->g = t; rgb->b = p;
            break;
        case 1:
            rgb->r = q; rgb->g = hsv->v; rgb->b = p;
            break;
        case 2:
            rgb->r = p; rgb->g = hsv->v; rgb->b = t;
            break;
        case 3:
            rgb->r = p; rgb->g = q; rgb->b = hsv->v;
            break;
        case 4:
            rgb->r = t; rgb->g = p; rgb->b = hsv->v;
            break;
        default:
            rgb->r = hsv->v; rgb->g = p; rgb->b = q;
            break;
    }

}
/*
HsvColor RgbToHsv(RgbColor rgb)
{
    HsvColor hsv;
    uint8_t rgbMin, rgbMax;

    rgbMin = rgb.r < rgb.g ? (rgb.r < rgb.b ? rgb.r : rgb.b) : (rgb.g < rgb.b ? rgb.g : rgb.b);
    rgbMax = rgb.r > rgb.g ? (rgb.r > rgb.b ? rgb.r : rgb.b) : (rgb.g > rgb.b ? rgb.g : rgb.b);

    hsv.v = rgbMax;
    if (hsv.v == 0)
    {
        hsv.h = 0;
        hsv.s = 0;
        return hsv;
    }

    hsv.s = 255 * uint16_t(rgbMax - rgbMin) / hsv.v;
    if (hsv.s == 0)
    {
        hsv.h = 0;
        return hsv;
    }

    if (rgbMax == rgb.r)
        hsv.h = 0 + 43 * (rgb.g - rgb.b) / (rgbMax - rgbMin);
    else if (rgbMax == rgb.g)
        hsv.h = 85 + 43 * (rgb.b - rgb.r) / (rgbMax - rgbMin);
    else
        hsv.h = 171 + 43 * (rgb.r - rgb.g) / (rgbMax - rgbMin);

    return hsv;
}

*/