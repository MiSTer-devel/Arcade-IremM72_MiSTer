#include "sim_state.h"

#include <cstdio>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <sys/stat.h>
#include <dirent.h>

// The M72 core has no savestate machinery yet.  This manager keeps the
// directory/naming plumbing alive so the UI and the state.* protocol methods
// keep working, but save/restore always fail with a clear reason.

const char *SimState::UnsupportedReason()
{
    return "savestates are not supported by the M72 core yet";
}

void SimState::SetGameName(const char *gameName)
{
    mGameName = gameName ? gameName : "";
    EnsureStateDirectory();
}

void SimState::EnsureStateDirectory()
{
    mkdir("states", 0755);
    if (!mGameName.empty())
    {
        std::string gameDir = "states/" + mGameName;
        mkdir(gameDir.c_str(), 0755);
    }
}

std::string SimState::GetStatePath(const char *filename)
{
    if (mGameName.empty())
        return std::string("states/") + filename;
    return "states/" + mGameName + "/" + filename;
}

bool SimState::SaveState(const char *filename)
{
    (void)filename;
    printf("SaveState: %s\n", UnsupportedReason());
    return false;
}

bool SimState::RestoreState(const char *filename)
{
    (void)filename;
    printf("RestoreState: %s\n", UnsupportedReason());
    return false;
}

std::vector<std::string> SimState::GetStateFiles()
{
    std::vector<std::string> files;

    std::string dirPath = mGameName.empty() ? "states" : ("states/" + mGameName);
    DIR *dir = opendir(dirPath.c_str());
    if (!dir)
        return files;

    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr)
    {
        std::string filename = entry->d_name;
        if (filename.size() > 9 && filename.substr(filename.size() - 9) == ".m72state")
        {
            files.push_back(filename);
        }
    }
    closedir(dir);

    std::sort(files.begin(), files.end());
    return files;
}

std::string SimState::GenerateNextStateName()
{
    std::vector<std::string> existingFiles = GetStateFiles();

    for (int nextNum = 0; nextNum < 1000; nextNum++)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(3) << nextNum << ".m72state";
        if (std::find(existingFiles.begin(), existingFiles.end(), ss.str()) == existingFiles.end())
        {
            return ss.str();
        }
    }

    return "999.m72state";
}
