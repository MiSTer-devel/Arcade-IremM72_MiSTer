#include "file_search.h"
#include <fstream>
#include <filesystem>
#include <cstdio>

namespace fs = std::filesystem;

// Define the global FileSearch instance
FileSearch gFileSearch;

FileSearch::FileSearch()
{
    // Constructor - nothing to initialize
}

FileSearch::~FileSearch()
{
    // Clean up any open zip archives
    for (auto &[path, zipInfo] : mZipFiles)
    {
        delete zipInfo;
    }
    mZipFiles.clear();
}

bool FileSearch::AddSearchPath(const std::string &path)
{
    if (!fs::exists(path))
    {
        printf("Search path does not exist: %s\n", path.c_str());
        return false;
    }

    if (fs::is_directory(path))
    {
        // Add directory to the search list
        SearchPath searchPath;
        searchPath.mPath = path;
        searchPath.mType = PathType::DIRECTORY;
        mSearchPaths.push_back(searchPath);
        printf("Added directory to search path: %s\n", path.c_str());
        return true;
    }
    else if (fs::is_regular_file(path))
    {
        // Check if it's a zip file by extension
        if (path.size() > 4 && path.substr(path.size() - 4) == ".zip")
        {
            // Create and initialize the zip archive
            ZipInfo *zipInfo = new ZipInfo();

            // Try to open the zip file
            if (!mz_zip_reader_init_file(&zipInfo->mArchive, path.c_str(), 0))
            {
                printf("Failed to open zip file: %s\n", path.c_str());
                delete zipInfo;
                return false;
            }

            zipInfo->mValid = true;
            mZipFiles[path] = zipInfo;

            // Add to the ordered search list
            SearchPath searchPath;
            searchPath.mPath = path;
            searchPath.mType = PathType::ZIP_FILE;
            mSearchPaths.push_back(searchPath);

            printf("Added zip file to search path: %s\n", path.c_str());
            return true;
        }
        else
        {
            printf("Path is a file but not a zip: %s\n", path.c_str());
            return false;
        }
    }

    printf("Path is neither a directory nor a file: %s\n", path.c_str());
    return false;
}

void FileSearch::ClearSearchPaths()
{
    mSearchPaths.clear();

    // Clean up and clear zip files
    for (auto &[path, zipInfo] : mZipFiles)
    {
        delete zipInfo;
    }
    mZipFiles.clear();
}

bool FileSearch::LoadFile(const std::string &filename, std::vector<uint8_t> &buffer)
{
    // Search all paths in the order they were added
    for (const auto &searchPath : mSearchPaths)
    {
        if (searchPath.mType == PathType::DIRECTORY)
        {
            if (LoadFromDirectory(searchPath.mPath, filename, buffer))
            {
                return true;
            }
        }
        else if (searchPath.mType == PathType::ZIP_FILE)
        {
            if (LoadFromZip(searchPath.mPath, filename, buffer))
            {
                return true;
            }
        }
    }

    // File not found in any search path
    return false;
}

bool FileSearch::LoadFromDirectory(const std::string &dirPath, const std::string &filename, std::vector<uint8_t> &buffer)
{
    fs::path filePath = fs::path(dirPath) / filename;

    // Check if file exists
    if (!fs::exists(filePath))
    {
        return false;
    }

    // Open the file
    std::ifstream file(filePath, std::ios::binary);
    if (!file)
    {
        printf("Failed to open file: %s\n", filePath.c_str());
        return false;
    }

    // Get the file size
    file.seekg(0, std::ios::end);
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    // Resize buffer and read file
    buffer.resize(size);
    if (!file.read(reinterpret_cast<char *>(buffer.data()), size))
    {
        printf("Failed to read file: %s\n", filePath.c_str());
        return false;
    }

    printf("Loaded file from directory: %s\n", filePath.c_str());
    return true;
}

bool FileSearch::LoadFromZip(const std::string &zipPath, const std::string &filename, std::vector<uint8_t> &buffer)
{
    ZipInfo *zipInfo = mZipFiles[zipPath];
    if (!zipInfo || !zipInfo->mValid)
    {
        return false;
    }

    // Find the file inside the zip archive
    int fileIndex = mz_zip_reader_locate_file(&zipInfo->mArchive, filename.c_str(), nullptr, 0);
    if (fileIndex < 0)
    {
        return false;
    }

    // Get file info
    mz_zip_archive_file_stat fileStat;
    if (!mz_zip_reader_file_stat(&zipInfo->mArchive, fileIndex, &fileStat))
    {
        printf("Failed to get file info from zip: %s -> %s\n", zipPath.c_str(), filename.c_str());
        return false;
    }

    // Resize buffer to fit the file
    buffer.resize(fileStat.m_uncomp_size);

    // Extract file
    if (!mz_zip_reader_extract_to_mem(&zipInfo->mArchive, fileIndex, buffer.data(), buffer.size(), 0))
    {
        printf("Failed to extract file from zip: %s -> %s\n", zipPath.c_str(), filename.c_str());
        return false;
    }

    printf("Loaded file from zip: %s -> %s\n", zipPath.c_str(), filename.c_str());
    return true;
}

bool FileSearch::LoadFileByCRC(uint32_t crc32, std::vector<uint8_t> &buffer)
{
    // Search through all zip files in search paths
    for (const auto &searchPath : mSearchPaths)
    {
        if (searchPath.mType == PathType::ZIP_FILE)
        {
            if (LoadFromZipByCRC(searchPath.mPath, crc32, buffer))
            {
                return true;
            }
        }
    }

    return false;
}

bool FileSearch::LoadFromZipByCRC(const std::string &zipPath, uint32_t crc32, std::vector<uint8_t> &buffer)
{
    ZipInfo *zipInfo = mZipFiles[zipPath];
    if (!zipInfo || !zipInfo->mValid)
    {
        return false;
    }

    // Search for file by CRC32
    for (unsigned int fileIndex = 0; fileIndex < zipInfo->mArchive.m_total_files; fileIndex++)
    {
        mz_zip_archive_file_stat fileStat;
        if (mz_zip_reader_file_stat(&zipInfo->mArchive, fileIndex, &fileStat))
        {
            if (fileStat.m_crc32 == crc32)
            {
                // Found file with matching CRC
                buffer.resize(fileStat.m_uncomp_size);

                if (mz_zip_reader_extract_to_mem(&zipInfo->mArchive, fileIndex, buffer.data(), buffer.size(), 0))
                {
                    return true;
                }
                else
                {
                    buffer.clear();
                    return false;
                }
            }
        }
    }

    return false;
}

std::string FileSearch::FindFilePath(const std::string &filename)
{
    // Search through all search paths
    for (const auto &searchPath : mSearchPaths)
    {
        if (searchPath.mType == PathType::DIRECTORY)
        {
            std::filesystem::path fullPath = std::filesystem::path(searchPath.mPath) / filename;
            if (std::filesystem::exists(fullPath))
            {
                return fullPath.string();
            }
        }
        else if (searchPath.mType == PathType::ZIP_FILE)
        {
            // For ZIP files, return the ZIP path if the file exists inside it
            ZipInfo *zipInfo = mZipFiles[searchPath.mPath];
            if (zipInfo && zipInfo->mValid)
            {
                int fileIndex = mz_zip_reader_locate_file(&zipInfo->mArchive, filename.c_str(), nullptr, 0);
                if (fileIndex >= 0)
                {
                    return searchPath.mPath; // Return the ZIP file path
                }
            }
        }
    }

    return ""; // File not found
}

std::vector<FileSearch::SearchPath> FileSearch::SaveSearchPaths() const
{
    return mSearchPaths;
}

void FileSearch::RestoreSearchPaths(const std::vector<SearchPath> &savedPaths)
{
    // Clear current paths and zip cache
    ClearSearchPaths();

    // Restore saved paths
    for (const auto &path : savedPaths)
    {
        AddSearchPath(path.mPath);
    }
}
