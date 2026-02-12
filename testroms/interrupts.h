#if !defined( INTERRUPTS_H )
#define INTERRUPTS_H 1

#include "util.h"

__attribute__((interrupt)) void __far vblank_handler();
__attribute__((interrupt)) void __far dma_done_handler();
__attribute__((interrupt)) void __far hint_handler();
__attribute__((interrupt)) void __far audio_io_handler();
__attribute__((interrupt)) void __far generic_handler();

static inline void enable_interrupts() { asm( "sti" ); }
static inline void disable_interrupts() { asm( "cli" ); }

#endif