#ifndef SIM_SOUND_UI_H
#define SIM_SOUND_UI_H 1

#include <cstdint>

// Sound command monitor.  The main CPU talks to the Z80 sound board through a
// single byte-wide latch (I/O port 0x00, `snd_latch1_wr` in rtl/pal.sv); M72
// boards have a second latch at port 0xC0 that pulls the Z80's NMI.  These
// hooks watch both strobes and log every byte the game sends, decoded against
// the sound driver's own command table in the shared sound RAM.

// Called once per CLK_32M tick from SimCore::TickOneCycle.
void SoundCommandsTick();

// Queue a command byte to be poked into the sound latch as if the main CPU had
// written it (latch1 = I/O port 0x00, latch2 = port 0xC0).  Lets a sound be
// auditioned without playing the game to the point that triggers it.
void SoundCommandsInject(uint8_t value, bool latch2 = false);

// Called when a game is loaded or the core is reset.
void SoundCommandsReset();

#endif // SIM_SOUND_UI_H
