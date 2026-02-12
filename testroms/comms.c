#include <stdint.h>
#include <stdbool.h>

#include "printf/printf.h"

#include "util.h"
#include "comms.h"

typedef struct
{
    uint8_t v;
    uint8_t __hi;
} u8_rom;

typedef struct
{
    uint8_t v;
    uint8_t __hi[7];
} u32_rom;

typedef volatile struct CommsRegisters
{
    u8_rom magic[4];

    u32_rom active;
    u32_rom pending;
    u32_rom in_byte;
    u32_rom in_seq;
    u32_rom out_seq;

    uint16_t reserved[256 - (6 * 4)];

    uint16_t out_area[256];
} CommsRegisters;
_Static_assert(sizeof(CommsRegisters) == 0x400, "CommsRegisters size mismatch");

__far CommsRegisters *comms_regs = (__far CommsRegisters *)0x3000fc00;

static bool magic_valid = false;
static bool comms_active = false;

uint8_t comms_in_seq;
uint8_t comms_out_seq;
volatile uint8_t dummy_read;

static bool comms_check_magic()
{ 
    if(!magic_valid)
    {
        if( comms_regs->magic[0].v == 'P' && comms_regs->magic[1].v == 'I' && comms_regs->magic[2].v == 'C' && comms_regs->magic[3].v == 'O' )
        {
            magic_valid = true;
        }
    }

    return magic_valid;
}

bool comms_check_active()
{
    if( comms_regs->active.v == 1 ) return comms_check_magic();
    return false;
}


bool comms_update()
{
    bool active = comms_check_active();

    if (!comms_active && active)
    {
        comms_active = true;
        comms_in_seq = 0;
        comms_out_seq = 0;
    }
    else if (comms_active && !active)
    {
        comms_active = false;
    }

    return active;
}

void comms_status(char *str, int len)
{
    snprintf(str, len, "ACT: %01X IN: %02X/%02X OUT: %02X/%02X  ", comms_regs->active.v, comms_regs->in_seq.v, comms_in_seq, comms_regs->out_seq.v, comms_out_seq);
}

int comms_read(void *buffer, int maxlen)
{
    if (!comms_update()) return 0;

    int len = 0;

    uint8_t *buffer8 = (uint8_t *)buffer;

    while(comms_regs->active.v && ( (comms_in_seq != comms_regs->in_seq.v) || comms_regs->pending.v ))
    { 
        if(comms_in_seq != comms_regs->in_seq.v)
        {
            buffer8[len] = comms_regs->in_byte.v;
            comms_in_seq++;
            len++;

            if (len == maxlen) return len;
        }
    }
    return len;
}

int comms_write(const __far void *data, int len)
{
    int sent = 0;

    if (!comms_update()) return 0;

    const __far uint8_t *data8 = (const __far uint8_t *)data;

    while (sent < len)
    {
        const uint8_t b = data8[sent];
        dummy_read = comms_regs->out_area[b];
        comms_out_seq++;
        while (comms_regs->out_seq.v != comms_out_seq)
        {
            if( !comms_regs->active.v )
            {
                break;
            }
        };
        sent++;
    }

    return sent;
}
