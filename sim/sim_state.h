#pragma once

#include <vector>
#include <string>

// Save-state manager.  The M72 core does not implement savestate machinery
// yet, so every operation fails with a clear "unsupported" error; the UI and
// protocol surface stay in place for when the core grows ss support.
class SimState
{
  public:
    SimState() = default;

    // Set the current game name for directory organization
    void SetGameName(const char *gameName);

    // Save state to the specified file (always fails: unsupported by core)
    bool SaveState(const char *filename);

    // Restore state from the specified file (always fails: unsupported by core)
    bool RestoreState(const char *filename);

    // Get list of all available state files in game-specific directory
    std::vector<std::string> GetStateFiles();

    // Get the full path for a state file
    std::string GetStatePath(const char *filename);

    // Create state directory if it doesn't exist
    void EnsureStateDirectory();

    // Generate next available state filename (000.m72state, 001.m72state, ...)
    std::string GenerateNextStateName();

    // Human-readable reason save/restore is unavailable
    static const char *UnsupportedReason();

  private:
    std::string mGameName;
};
