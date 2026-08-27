#pragma once

#include <vector>
#include <string>

// Forward declarations
class M72;
class SimDDR;

// Save-state manager: drives the core's ss_do_save/ss_do_restore handshake
// and dumps/loads the DDR slot window to .m72state files.
class SimState
{
  public:
    SimState(M72 *top, SimDDR *memory, int offset, int size);

    // Set the current game name for directory organization
    void SetGameName(const char *gameName);

    // Save state to the specified file
    bool SaveState(const char *filename);

    // Restore state from the specified file
    bool RestoreState(const char *filename);

    // Get list of all available state files in game-specific directory
    std::vector<std::string> GetStateFiles();

    // Get the full path for a state file
    std::string GetStatePath(const char *filename);

    // Create state directory if it doesn't exist
    void EnsureStateDirectory();

    // Generate next available state filename (000.m72state, 001.m72state, ...)
    std::string GenerateNextStateName();

  private:
    M72 *mTop;
    SimDDR *mMemory;
    int mOffset;
    int mSize;
    std::string mGameName;
};
