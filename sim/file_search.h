#ifndef FILE_SEARCH_H
#define FILE_SEARCH_H

#include <string>
#include <vector>
#include <filesystem>
#include <unordered_map>
#include "third_party/miniz.h"

/**
 * Class for searching and loading files from various paths.
 * Supports searching in directories and inside zip files.
 * Search is performed in the order paths were added.
 */
class FileSearch
{
  public:
    // Enum to track search path type
    enum class PathType
    {
        DIRECTORY,
        ZIP_FILE
    };

    // Structure to track search paths in order
    struct SearchPath
    {
        std::string mPath;
        PathType mType;
    };

    // Constructor
    FileSearch();

    // Destructor - clean up cached zip files
    ~FileSearch();

    /**
     * Add a path to the search list
     * @param path Path to a directory or zip file
     * @return true if path exists and was added
     */
    bool AddSearchPath(const std::string &path);

    /**
     * Load a file into the provided buffer
     * @param filename Name of the file to locate
     * @param buffer Vector to store the file contents
     * @return true if file was found and loaded
     */
    bool LoadFile(const std::string &filename, std::vector<uint8_t> &buffer);

    /**
     * Load a file by CRC32 from zip files in search paths
     * @param crc32 CRC32 value to search for
     * @param buffer Vector to store the file contents
     * @return true if file was found and loaded
     */
    bool LoadFileByCRC(uint32_t crc32, std::vector<uint8_t> &buffer);

    /**
     * Find the full path of a file in search paths
     * @param filename Name of the file to locate
     * @return Full path to the file, or empty string if not found
     */
    std::string FindFilePath(const std::string &filename);

    /**
     * Clear all search paths
     */
    void ClearSearchPaths();

    /**
     * Save current search paths state
     * @return Saved state that can be restored later
     */
    std::vector<SearchPath> SaveSearchPaths() const;

    /**
     * Restore search paths from saved state
     * @param savedPaths Previously saved search paths
     */
    void RestoreSearchPaths(const std::vector<SearchPath> &savedPaths);

    /**
     * Get the number of search paths
     * @return Number of active search paths
     */
    size_t GetSearchPathCount() const
    {
        return mSearchPaths.size();
    }

  private:
    // Structure to hold opened zip archive information
    struct ZipInfo
    {
        mz_zip_archive mArchive;
        bool mValid;

        ZipInfo() : mValid(false)
        {
            mz_zip_zero_struct(&mArchive);
        }

        ~ZipInfo()
        {
            if (mValid)
            {
                mz_zip_reader_end(&mArchive);
            }
        }
    };

    // List of search paths in the order they were added
    std::vector<SearchPath> mSearchPaths;

    // Map of zip file paths to their archive objects
    std::unordered_map<std::string, ZipInfo *> mZipFiles;

    // Try to load file from a directory
    bool LoadFromDirectory(const std::string &dirPath, const std::string &filename, std::vector<uint8_t> &buffer);

    // Try to load file from a zip archive
    bool LoadFromZip(const std::string &zipPath, const std::string &filename, std::vector<uint8_t> &buffer);

    // Try to load file by CRC from a zip archive
    bool LoadFromZipByCRC(const std::string &zipPath, uint32_t crc32, std::vector<uint8_t> &buffer);
};

// Global FileSearch instance that can be used throughout the application
extern FileSearch gFileSearch;

#endif // FILE_SEARCH_H
