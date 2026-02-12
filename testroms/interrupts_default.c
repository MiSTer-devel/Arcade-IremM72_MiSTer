#include "interrupts.h"

__attribute__((interrupt,weak)) void __far vblank_handler()
{
	return;
}

__attribute__((interrupt,weak)) void __far dma_done_handler()
{
	return;
}

__attribute__((interrupt,weak)) void __far hint_handler()
{
	return;
}

__attribute__((interrupt,weak)) void __far audio_io_handler()
{
	return;
}

__attribute__((interrupt,weak)) void __far generic_handler()
{
	return;
}
