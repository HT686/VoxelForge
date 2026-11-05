#pragma once
// The look of VoxelForge: color tokens from the logo and the widgets every window uses. Everything
// that draws a control goes through here, so that the whole application changes in one place.
#include <imgui.h>
#include <map>
#include <string>
#include "icons.hpp"

namespace forge {
// Surfaces, from back to front.
constexpr ImVec4 bg(.043f, .055f, .075f, 1), surface(.067f, .082f, .110f, 1), raised(.102f, .125f, .161f, 1), hover(.137f, .165f, .208f, 1), pressed(.169f, .200f, .251f, 1);
constexpr ImVec4 line(1, 1, 1, .075f), lineStrong(1, 1, 1, .14f);
constexpr ImVec4 text(.91f, .925f, .949f, 1), textDim(.60f, .64f, .70f, 1), textFaint(.40f, .44f, .50f, 1);
// Forge orange for the one main action, sky for what is selected, green and red for states.
constexpr ImVec4 orange(.969f, .627f, .255f, 1), orangeDeep(.953f, .459f, .184f, 1), sky(.310f, .765f, .969f, 1), green(.204f, .827f, .600f, 1), danger(.973f, .443f, .443f, 1), amber(.984f, .749f, .141f, 1);
inline ImU32 col(const ImVec4& c, float alpha = 1.f) { return ImGui::ColorConvertFloat4ToU32(ImVec4(c.x, c.y, c.z, c.w*alpha)); }
inline ImVec4 alpha(const ImVec4& c, float a) { return ImVec4(c.x, c.y, c.z, a); }

extern ImFont* fontBody; extern ImFont* fontMedium; extern ImFont* fontTitle; extern ImFont* fontIcon;
void loadFonts(float scale);
void applyTheme();
float px(float v);   // scaled by the screen

// Where each control was drawn in the last frame, so that VOXELFORGE_SCRIPT can click it like a user.
std::map<std::string, ImVec2>& buttonPositions();
void noteItem(const char* label);

// Buttons. A label may start with an icon; what follows ## stays hidden as usual.
bool button(const char* label, ImVec2 size = {0, 0});
bool primaryButton(const char* label, ImVec2 size = {0, 0});
bool dangerButton(const char* label, ImVec2 size = {0, 0});
bool toggleButton(const char* label, bool on, ImVec2 size = {0, 0});
// Icon only, with a tooltip. key: name for scripts; on: stays lit.
bool iconButton(const char* icon, const char* key, const char* tooltip, bool on = false, float size = 0);
// A large square with icon and name, for choosing a tool.
bool toolTile(const char* icon, const char* label, ImVec2 size, bool on = false);
// The same with a picture. Without one, a colored letter stands for it. dim: not available right now.
bool imageTile(unsigned texture, const char* letter, const ImVec4& color, const char* label, ImVec2 size, bool dim = false);
// One entry of the main navigation.
bool navItem(const char* icon, const char* label, bool on, float height);
bool segmented(const char* id, int& value, const char* const* names, int n);

// Inputs.
bool slider(const char* label, float* v, float lo, float hi, const char* format = "%.1f", bool logarithmic = false);
bool sliderInt(const char* label, int* v, int lo, int hi, const char* format = "%d");
bool toggle(const char* label, bool* v);
bool combo(const char* label, int* current, const char* const* names, int n);

// Layout. beginCard always needs endCard.
void beginCard(const char* id, const char* title = nullptr, const char* icon = nullptr);
void endCard();
bool collapsible(const char* label, bool defaultOpen = false);   // inside a panel, opens a section
void hint(const char* text);
void labelValue(const char* label, const char* value, const ImVec4& color = text);
void chip(const char* text, const ImVec4& color, const char* icon = nullptr);
void chipAt(ImDrawList* dl, ImVec2 pos, const char* text, const ImVec4& color, const char* icon = nullptr);
void spinner(ImDrawList* dl, ImVec2 center, float radius, const ImVec4& color);
// A floating bar over the 3D view. Pair with endFloating.
void beginFloating(const char* id, ImVec2 pos, ImVec2 size);
void endFloating();

// Short messages that fade away, bottom right.
enum class Toast { Info, Success, Warning, Error };
void toast(const std::string& text, Toast kind = Toast::Info);
void drawToasts();
// A large panel with a spinner over the view, for longer work.
void busyOverlay(ImVec2 pos, ImVec2 size, const char* title, float progress);
}
