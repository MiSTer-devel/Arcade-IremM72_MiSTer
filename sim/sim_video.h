#if !defined(SIM_VIDEO_H)
#define SIM_VIDEO_H 1

#include <stdint.h>
#include <SDL.h>
#include <string>

#include "imgui_wrap.h"

class SimVideo : public Window
{
  public:
    SimVideo() : Window("Video Output", ImGuiWindowFlags_NoScrollbar)
    {
    }

    ~SimVideo()
    {
        Deinit();
    }

    void Init()
    {
    }

    void Init(int w, int h, SDL_Renderer *renderer)
    {
        mWidth = w;
        mHeight = h;
        mPixels = new uint32_t[mWidth * mHeight];
        if (renderer)
        {
            mTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBX8888, SDL_TEXTUREACCESS_STREAMING, mWidth, mHeight);
        }
        else
        {
            mTexture = nullptr; // Headless mode
        }
        mX = 0;
        mY = 0;
        mRotated = false;
        mInVsync = false;
        mInHsync = false;
        mInCe = false;
    }

    void Deinit()
    {
        if (mPixels)
            delete[] mPixels;
        if (mTexture)
            SDL_DestroyTexture(mTexture);

        mPixels = nullptr;
        mTexture = nullptr;
    }

    void Clock(bool ce, bool hsync, bool vsync, uint8_t r, uint8_t g, uint8_t b)
    {
        if (!ce)
        {
            mInCe = false;
            return;
        }

        if (mInCe)
            return;

        if (hsync)
        {
            mX = 0;
            if (!mInHsync)
                mY++;
        }

        if (vsync)
        {
            if (!mInVsync)
                mX = 0;
            // -1, not 0: the hblank rising edge at the start of the first active line
            // increments mY, so it must land on row 0 (not 1).  Resetting to 0 here put
            // the first visible line in row 1 (row 0 garbage) and pushed the last line to
            // mY==mHeight, where the bounds check dropped it.
            mY = -1;
        }

        mInHsync = hsync;
        mInVsync = vsync;
        mInCe = ce;

        if (!hsync && !vsync)
        {
            uint32_t c = r << 24 | g << 16 | b << 8;
            if (mX >= 0 && mX < mWidth && mY >= 0 && mY < mHeight)
                mPixels[(mY * mWidth) + mX] = c;
            mX++;
        }
    }

    void UpdateTexture()
    {
        if (!mTexture)
            return; // Skip in headless mode

        SDL_Rect region;

        int lineCount = mHeight;
        int lineStart = 0;
        region.x = 0;
        region.y = lineStart;
        region.w = mWidth;
        region.h = lineCount;

        void *work;
        int pitch;

        SDL_LockTexture(mTexture, &region, &work, &pitch);

        for (int line = 0; line < lineCount; line++)
        {
            uint8_t *dest = ((uint8_t *)work) + (pitch * line);
            uint32_t *src = mPixels + ((line + lineStart) * mWidth);
            if (!mInVsync && ((line + lineStart) == mY))
            {
                memset(dest, 0x2f, pitch);
            }
            else
            {
                memcpy(dest, src, pitch);
            }
        }

        SDL_UnlockTexture(mTexture);
    }

    bool SaveScreenshot(const char *filename);
    std::string GenerateScreenshotFilename(const char *gameName);

    void Draw()
    {
        ImGui::Checkbox("TATE", &mRotated);
        ImGui::SameLine();
        ImGui::Checkbox("Flip X", &mFlipX);
        ImGui::SameLine();
        ImGui::Checkbox("Flip Y", &mFlipY);
        ImGui::SameLine();
        ImGui::Text("X: %03d Y: %03d", mX, mY);

        if (!mScreenshotStatus.empty())
        {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0, 1, 0, 1), "%s", mScreenshotStatus.c_str());
            if (--mScreenshotStatusTimer <= 0)
                mScreenshotStatus.clear();
        }

        ImVec2 availSize = ImGui::GetContentRegionAvail();
        int w = availSize.x;
        int h = mRotated ? ((w * 4) / 3) : ((w * 3) / 4);

        ImGuiWindow *window = ImGui::GetCurrentWindow();
        if (!window->SkipItems)
        {

            const ImRect bb(window->DC.CursorPos, window->DC.CursorPos + ImVec2(w, h));
            ImGui::ItemSize(bb);
            if (ImGui::ItemAdd(bb, 0))
            {
                // Render
                ImVec2 uv0, uv1, uv2, uv3;

                if (mRotated)
                {
                    uv0 = ImVec2(1, 0);
                    uv1 = ImVec2(1, 1);
                    uv2 = ImVec2(0, 1);
                    uv3 = ImVec2(0, 0);
                }
                else
                {
                    uv0 = ImVec2(0, 0);
                    uv1 = ImVec2(1, 0);
                    uv2 = ImVec2(1, 1);
                    uv3 = ImVec2(0, 1);
                }

                window->DrawList->AddImageQuad((ImTextureID)mTexture, bb.GetTL(), bb.GetTR(), bb.GetBR(), bb.GetBL(), uv0, uv1, uv2, uv3,
                                               ImGui::GetColorU32(ImVec4(1, 1, 1, 1)));
            }
        }
    }

    int mWidth, mHeight;
    uint32_t *mPixels = nullptr;

    bool mRotated;

    // Global screen-flip checkboxes; driven into sim_top->flip_x / flip_y each tick.
    bool mFlipX = false;
    bool mFlipY = false;

    int mX, mY;
    bool mInHsync, mInVsync, mInCe;
    SDL_Texture *mTexture = nullptr;

    std::string mScreenshotStatus;
    int mScreenshotStatusTimer = 0;
};

#endif
