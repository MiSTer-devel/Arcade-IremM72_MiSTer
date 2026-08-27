
#ifndef SIM_UI_H
#define SIM_UI_H

void UiInit(const char *title);
void UiInitWindows();

void UiGameChanged();

bool UiBeginFrame();
void UiEndFrame();
void UiDraw();

#endif // SIM_UI_H
