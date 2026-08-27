#include "sim_video.h"
#include <ctime>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

bool SimVideo::SaveScreenshot(const char *filename)
{
    if (!mPixels || !mWidth || !mHeight)
        return false;

    int actualWidth = mWidth;
    int actualHeight = mHeight;

    if (mRotated)
    {
        std::swap(actualWidth, actualHeight);
    }

    uint8_t *rgbData = new uint8_t[actualWidth * actualHeight * 3];

    for (int y = 0; y < actualHeight; y++)
    {
        for (int x = 0; x < actualWidth; x++)
        {
            uint32_t pixel;

            if (mRotated)
            {
                int srcX = actualHeight - 1 - y;
                int srcY = x;
                pixel = mPixels[srcY * mWidth + srcX];
            }
            else
            {
                pixel = mPixels[y * mWidth + x];
            }

            int dstIdx = (y * actualWidth + x) * 3;
            rgbData[dstIdx + 0] = (pixel >> 24) & 0xFF;
            rgbData[dstIdx + 1] = (pixel >> 16) & 0xFF;
            rgbData[dstIdx + 2] = (pixel >> 8) & 0xFF;
        }
    }

    int result = stbi_write_png(filename, actualWidth, actualHeight, 3, rgbData, actualWidth * 3);

    delete[] rgbData;

    if (result)
    {
        mScreenshotStatus = "Screenshot saved";
        mScreenshotStatusTimer = 120;
    }
    else
    {
        mScreenshotStatus = "Screenshot failed";
        mScreenshotStatusTimer = 120;
    }

    return result != 0;
}

std::string SimVideo::GenerateScreenshotFilename(const char *gameName)
{
    mkdir("screenshots", 0755);

    time_t now = time(nullptr);
    struct tm *timeinfo = localtime(&now);

    std::stringstream filename;
    filename << "screenshots/" << gameName << "_" << std::put_time(timeinfo, "%Y%m%d_%H%M%S") << ".png";

    return filename.str();
}
