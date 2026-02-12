#include "input.h"
#include "util.h"

static uint16_t s_prev = 0;
static uint16_t s_cur = 0;

void input_update()
{
    uint16_t p1p2 = __inw(0x00);
    uint16_t coin = __inw(0x02);

    s_prev = s_cur;
    s_cur = (p1p2 & 0x00ff) | ((coin & 0x00ff) << 8);
}

bool input_down(InputKey key)
{
    return (s_cur & key) != key;
}

bool input_released(InputKey key)
{
    return ((s_cur & key) != 0) && (((s_prev ^ s_cur) & key) != 0);
}

bool input_pressed(InputKey key)
{
    return ((s_cur & key) != key) && (((s_prev ^ s_cur) & key) != 0);
}
