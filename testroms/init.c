#include <stdint.h>
#include <stddef.h>

#include "util.h"

#include "interrupts.h"

__attribute__ ((section(".vectors"))) __attribute__((used))
static const __far void * exception_vectors[256] =
{
	generic_handler,
	generic_handler,
	generic_handler,
	generic_handler,
	generic_handler,
	generic_handler,
	generic_handler,
	generic_handler,
	vblank_handler,
	dma_done_handler,
	hint_handler,
	audio_io_handler,
	generic_handler,
	generic_handler,
	generic_handler,
	generic_handler,
};

void putchar_(char c) {}
