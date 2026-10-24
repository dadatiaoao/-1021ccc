#ifndef LCD_ORIENTATION_H
#define LCD_ORIENTATION_H
#include <stdint.h>
/* Drawing and touch share the installed panel orientation. */
#ifndef LCD_MOUNT_ROTATE_180
#define LCD_MOUNT_ROTATE_180 1
#endif
#define UI_SCREEN_WIDTH 480u
#define UI_SCREEN_HEIGHT 272u
#if LCD_MOUNT_ROTATE_180
#define LCD_TOUCH_X(x) ((int32_t)UI_SCREEN_WIDTH - 1 - (x))
#define LCD_TOUCH_Y(y) ((int32_t)UI_SCREEN_HEIGHT - 1 - (y))
#else
#define LCD_TOUCH_X(x) (x)
#define LCD_TOUCH_Y(y) (y)
#endif
static __inline int Lcd_OrientRect(uint16_t *x, uint16_t *y, uint16_t *w,
    uint16_t *h, uint16_t screen_w, uint16_t screen_h)
{
    if (!*w || !*h || *x>=screen_w || *y>=screen_h) return 0;
    if (*w>screen_w-*x) *w=(uint16_t)(screen_w-*x);
    if (*h>screen_h-*y) *h=(uint16_t)(screen_h-*y);
#if LCD_MOUNT_ROTATE_180
    *x=(uint16_t)(screen_w-*x-*w);
    *y=(uint16_t)(screen_h-*y-*h);
#endif
    return 1;
}
#endif
