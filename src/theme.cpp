#include <imgui_internal.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include "font_data.hpp"
#include "i18n.hpp"
#include "theme.hpp"

namespace forge {
ImFont* fontBody = nullptr; ImFont* fontMedium = nullptr; ImFont* fontTitle = nullptr; ImFont* fontIcon = nullptr;
static float gScale = 1;
float px(float v) { return v*gScale; }

std::map<std::string, ImVec2>& buttonPositions() { static std::map<std::string, ImVec2> m; return m; }
void noteItem(const char* label) { const ImVec2 a = ImGui::GetItemRectMin(), b = ImGui::GetItemRectMax(); buttonPositions()[label] = {(a.x+b.x)/2, (a.y+b.y)/2}; }

void loadFonts(float scale) {
    gScale = scale; ImGuiIO& io = ImGui::GetIO();
    static const ImWchar iconRange[] = {kIconMin, kIconMax, 0};
    static const ImWchar textRange[] = {0x20, 0xFF, 0x2013, 0x2014, 0x2018, 0x201E, 0x2022, 0x2022, 0x2026, 0x2026, 0x20AC, 0x20AC, 0};
    auto load = [&](const unsigned char* data, int size, float pxSize, float iconSize) {
        ImFontConfig c; c.OversampleH = 3; c.OversampleV = 2; c.PixelSnapH = false; c.RasterizerDensity = 1;
        ImFont* f = io.Fonts->AddFontFromMemoryCompressedTTF(data, size, pxSize*scale, &c, textRange);
        ImFontConfig m; m.MergeMode = true; m.OversampleH = 2; m.OversampleV = 2; m.PixelSnapH = true; m.GlyphOffset = {0, 2.0f*scale*iconSize/16}; m.GlyphMinAdvanceX = iconSize*scale;
        io.Fonts->AddFontFromMemoryCompressedTTF(font_lucide_sub_compressed_data, font_lucide_sub_compressed_size, iconSize*scale, &m, iconRange);
        return f;
    };
    fontBody = load(font_inter_Regular_compressed_data, font_inter_Regular_compressed_size, 14.5f, 15.5f);
    fontMedium = load(font_inter_Medium_compressed_data, font_inter_Medium_compressed_size, 14.5f, 15.5f);
    fontTitle = load(font_inter_SemiBold_compressed_data, font_inter_SemiBold_compressed_size, 15.5f, 16.5f);
    fontIcon = load(font_inter_Medium_compressed_data, font_inter_Medium_compressed_size, 13.f, 21.f);
    io.FontDefault = fontBody;
}
void applyTheme() {
    ImGui::StyleColorsDark();
    ImGuiStyle& st = ImGui::GetStyle(); ImVec4* c = st.Colors;
    st.WindowRounding = 0; st.ChildRounding = px(12); st.FrameRounding = px(8); st.PopupRounding = px(10); st.GrabRounding = px(8); st.ScrollbarRounding = px(8); st.TabRounding = px(8);
    st.WindowBorderSize = 0; st.ChildBorderSize = 0; st.PopupBorderSize = 1; st.FrameBorderSize = 0;
    st.WindowPadding = {px(14), px(14)}; st.FramePadding = {px(10), px(7)}; st.ItemSpacing = {px(8), px(8)}; st.ItemInnerSpacing = {px(8), px(6)}; st.CellPadding = {px(6), px(5)};
    st.ScrollbarSize = px(9); st.GrabMinSize = px(12); st.IndentSpacing = px(14); st.SeparatorTextBorderSize = 1; st.SeparatorTextPadding = {px(0), px(4)};
    st.ButtonTextAlign = {.5f, .5f}; st.SelectableTextAlign = {0, .5f}; st.WindowTitleAlign = {0, .5f}; st.DisabledAlpha = .42f;
    c[ImGuiCol_WindowBg] = surface; c[ImGuiCol_ChildBg] = ImVec4(0, 0, 0, 0); c[ImGuiCol_PopupBg] = raised; c[ImGuiCol_Border] = line; c[ImGuiCol_Separator] = line; c[ImGuiCol_SeparatorHovered] = lineStrong; c[ImGuiCol_SeparatorActive] = sky;
    c[ImGuiCol_Text] = text; c[ImGuiCol_TextDisabled] = textDim;
    c[ImGuiCol_FrameBg] = bg; c[ImGuiCol_FrameBgHovered] = raised; c[ImGuiCol_FrameBgActive] = raised;
    c[ImGuiCol_Button] = raised; c[ImGuiCol_ButtonHovered] = hover; c[ImGuiCol_ButtonActive] = pressed;
    c[ImGuiCol_Header] = hover; c[ImGuiCol_HeaderHovered] = hover; c[ImGuiCol_HeaderActive] = pressed;
    c[ImGuiCol_CheckMark] = sky; c[ImGuiCol_SliderGrab] = sky; c[ImGuiCol_SliderGrabActive] = text;
    c[ImGuiCol_TitleBg] = raised; c[ImGuiCol_TitleBgActive] = raised; c[ImGuiCol_TitleBgCollapsed] = raised;
    c[ImGuiCol_ScrollbarBg] = ImVec4(0, 0, 0, 0); c[ImGuiCol_ScrollbarGrab] = hover; c[ImGuiCol_ScrollbarGrabHovered] = pressed; c[ImGuiCol_ScrollbarGrabActive] = lineStrong;
    c[ImGuiCol_TableHeaderBg] = ImVec4(0, 0, 0, 0); c[ImGuiCol_TableBorderLight] = line; c[ImGuiCol_TableBorderStrong] = line; c[ImGuiCol_TableRowBgAlt] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_ModalWindowDimBg] = ImVec4(.02f, .03f, .05f, .66f); c[ImGuiCol_TextSelectedBg] = alpha(sky, .35f); c[ImGuiCol_NavHighlight] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_PlotHistogram] = sky;
}

static float animate(ImGuiID id, bool target, float speed = 14.f) {
    // Soft transitions: each control keeps a small state between frames.
    static std::map<ImGuiID, float> values; float& v = values[id];
    const float dt = ImGui::GetIO().DeltaTime; v += ((target ? 1.f : 0.f)-v)*std::min(1.f, dt*speed);
    return v;
}
static ImVec4 mix(const ImVec4& a, const ImVec4& b, float t) { return ImVec4(a.x+(b.x-a.x)*t, a.y+(b.y-a.y)*t, a.z+(b.z-a.z)*t, a.w+(b.w-a.w)*t); }
static const char* visibleEnd(const char* label) { return ImGui::FindRenderedTextEnd(label); }

bool button(const char* label, ImVec2 size) { const bool r = ImGui::Button(tr(label), size); noteItem(label); return r; }
bool primaryButton(const char* label, ImVec2 size) {
    const char* const rawLabel = label; (void)rawLabel; label = tr(label);
    ImGuiWindow* w = ImGui::GetCurrentWindow(); if (w->SkipItems) return false;
    const ImVec2 ts = ImGui::CalcTextSize(label, nullptr, true); const ImGuiStyle& st = ImGui::GetStyle();
    const ImVec2 sz = ImGui::CalcItemSize(size, ts.x+st.FramePadding.x*2, ts.y+st.FramePadding.y*2);
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const bool r = ImGui::InvisibleButton(label, sz); noteItem(rawLabel);
    const bool hov = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled), act = ImGui::IsItemActive(); const float a = ImGui::GetStyle().Alpha;
    ImDrawList* dl = ImGui::GetWindowDrawList(); const ImVec2 q{p.x+sz.x, p.y+sz.y}; const float h = animate(ImGui::GetItemID(), hov && !act);
    if (hov && !act && a > .9f) for (int i = 3; i >= 1; i--) dl->AddRectFilled({p.x-i*1.5f, p.y-i*1.5f}, {q.x+i*1.5f, q.y+i*1.5f}, col(orange, .07f*h), st.FrameRounding+i*1.5f);
    const ImVec4 top = act ? orangeDeep : mix(orange, ImVec4(1, .72f, .36f, 1), h), bot = act ? orangeDeep : mix(orangeDeep, orange, h*.6f);
    // A vertical gradient inside rounded corners: draw the shape, then shade the vertices.
    const int v0 = dl->VtxBuffer.Size; dl->AddRectFilled(p, q, IM_COL32_WHITE, st.FrameRounding); const int v1 = dl->VtxBuffer.Size;
    ImGui::ShadeVertsLinearColorGradientKeepAlpha(dl, v0, v1, p, {p.x, q.y}, col(top, a), col(bot, a));
    ImGui::PushFont(fontMedium); const ImVec2 t2 = ImGui::CalcTextSize(label, nullptr, true);
    dl->AddText({p.x+(sz.x-t2.x)/2, p.y+(sz.y-t2.y)/2}, col(ImVec4(.07f, .05f, .03f, 1), a), label, visibleEnd(label)); ImGui::PopFont();
    return r;
}
bool dangerButton(const char* label, ImVec2 size) {
    ImGui::PushStyleColor(ImGuiCol_Button, alpha(danger, .16f)); ImGui::PushStyleColor(ImGuiCol_ButtonHovered, alpha(danger, .28f)); ImGui::PushStyleColor(ImGuiCol_ButtonActive, alpha(danger, .4f)); ImGui::PushStyleColor(ImGuiCol_Text, danger);
    const bool r = button(label, size); ImGui::PopStyleColor(4); return r;
}
bool toggleButton(const char* label, bool on, ImVec2 size) {
    if (on) { ImGui::PushStyleColor(ImGuiCol_Button, alpha(sky, .18f)); ImGui::PushStyleColor(ImGuiCol_ButtonHovered, alpha(sky, .26f)); ImGui::PushStyleColor(ImGuiCol_ButtonActive, alpha(sky, .34f)); ImGui::PushStyleColor(ImGuiCol_Text, sky); }
    const bool r = button(label, size); if (on) ImGui::PopStyleColor(4); return r;
}
bool iconButton(const char* icon, const char* key, const char* tooltip, bool on, float size) {
    const float s = size > 0 ? size : px(36); ImGui::PushID(key);
    const ImVec2 p = ImGui::GetCursorScreenPos(); const bool r = ImGui::InvisibleButton("##ib", {s, s}); noteItem(key);
    const bool hov = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled), act = ImGui::IsItemActive(); const float a = ImGui::GetStyle().Alpha;
    ImDrawList* dl = ImGui::GetWindowDrawList(); const ImVec2 q{p.x+s, p.y+s}; const float h = animate(ImGui::GetItemID(), hov);
    if (on) dl->AddRectFilled(p, q, col(sky, (.18f+.08f*h)*a), px(9)); else if (h > .01f || act) dl->AddRectFilled(p, q, col(act ? pressed : hover, std::max(h, act ? 1.f : 0.f)*a), px(9));
    ImGui::PushFont(fontIcon); const ImVec2 ts = ImGui::CalcTextSize(icon);
    dl->AddText({p.x+(s-ts.x)/2, p.y+(s-ts.y)/2+px(3)}, col(on ? sky : mix(textDim, text, h), a), icon); ImGui::PopFont();
    if (tooltip && hov && ImGui::GetCurrentContext()->HoveredIdTimer > .35f) { ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {px(10), px(7)}); ImGui::SetTooltip("%s", tr(tooltip)); ImGui::PopStyleVar(); }
    ImGui::PopID(); return r;
}
bool toolTile(const char* icon, const char* label, ImVec2 size, bool on) {
    const char* const rawLabel = label; (void)rawLabel; label = tr(label);
    ImGui::PushID(label); const ImVec2 p = ImGui::GetCursorScreenPos(); const bool r = ImGui::InvisibleButton("##tile", size); noteItem(rawLabel);
    const bool hov = ImGui::IsItemHovered(), act = ImGui::IsItemActive(); const float a = ImGui::GetStyle().Alpha; const float h = animate(ImGui::GetItemID(), hov);
    ImDrawList* dl = ImGui::GetWindowDrawList(); const ImVec2 q{p.x+size.x, p.y+size.y}; const float rnd = px(11);
    dl->AddRectFilled(p, q, col(on ? alpha(sky, .16f) : mix(raised, hover, act ? 1.f : h), a), rnd);
    dl->AddRect(p, q, col(on ? alpha(sky, .55f) : mix(line, lineStrong, h), a), rnd);
    ImGui::PushFont(fontIcon); const ImVec2 is = ImGui::CalcTextSize(icon); const ImVec2 ls = ImGui::CalcTextSize(label, nullptr, true);
    const float total = is.y+px(3)+ls.y, y0 = p.y+(size.y-total)/2;
    dl->AddText({p.x+(size.x-is.x)/2, y0}, col(on ? sky : mix(textDim, text, h), a), icon);
    dl->AddText({p.x+(size.x-ls.x)/2, y0+is.y+px(3)}, col(on ? sky : mix(textDim, text, .6f+.4f*h), a), label, visibleEnd(label)); ImGui::PopFont();
    ImGui::PopID(); return r;
}
bool navItem(const char* icon, const char* label, bool on, float height) {
    const char* const rawLabel = label; (void)rawLabel; label = tr(label);
    ImGui::PushID(label); ImGui::PushFont(fontMedium); const ImVec2 ls = ImGui::CalcTextSize(label); const ImVec2 is = ImGui::CalcTextSize(icon); const ImVec2 sz{ls.x+is.x+px(32), height};
    const ImVec2 p = ImGui::GetCursorScreenPos(); const bool r = ImGui::InvisibleButton("##nav", sz); noteItem(rawLabel);
    const bool hov = ImGui::IsItemHovered(); const float h = animate(ImGui::GetItemID(), hov), a = ImGui::GetStyle().Alpha; ImDrawList* dl = ImGui::GetWindowDrawList();
    if (on) dl->AddRectFilled(p, {p.x+sz.x, p.y+sz.y}, col(raised, a), px(9)); else if (h > .01f) dl->AddRectFilled(p, {p.x+sz.x, p.y+sz.y}, col(raised, h*.6f*a), px(9));
    if (on) dl->AddRectFilled({p.x+px(14), p.y+sz.y-px(2.5f)}, {p.x+sz.x-px(14), p.y+sz.y}, col(orange, a), px(2));
    const float y = p.y+(sz.y-ls.y)/2; dl->AddText({p.x+px(12), y}, col(on ? orange : mix(textDim, text, h), a), icon);
    dl->AddText({p.x+px(20)+is.x, y}, col(on ? text : mix(textDim, text, h), a), label);
    ImGui::PopFont(); ImGui::PopID(); return r;
}
bool segmented(const char* id, int& value, const char* const* names, int n) {
    ImGui::PushID(id); bool changed = false;
    const float w = ImGui::GetContentRegionAvail().x, h = ImGui::GetFrameHeight(); const ImVec2 p = ImGui::GetCursorScreenPos(); const float a = ImGui::GetStyle().Alpha;
    ImDrawList* dl = ImGui::GetWindowDrawList(); const float pad = px(3);
    dl->AddRectFilled(p, {p.x+w, p.y+h}, col(bg, a), px(9));
    const float cell = (w-2*pad)/n;
    // The lit part slides between the choices.
    static std::map<ImGuiID, float> slide; const ImGuiID sid = ImGui::GetID("slide"); auto it = slide.find(sid); if (it == slide.end()) it = slide.emplace(sid, (float)value).first;
    if (value >= 0) { it->second += ((float)value-it->second)*std::min(1.f, ImGui::GetIO().DeltaTime*18.f);
        const float x = p.x+pad+it->second*cell; dl->AddRectFilled({x, p.y+pad}, {x+cell, p.y+h-pad}, col(hover, a), px(7)); dl->AddRect({x, p.y+pad}, {x+cell, p.y+h-pad}, col(line, a), px(7)); }
    for (int i = 0; i < n; i++) {
        ImGui::SetCursorScreenPos({p.x+pad+i*cell, p.y}); ImGui::PushID(i);
        if (ImGui::InvisibleButton("##seg", {cell, h})) { changed = value != i; value = i; }
        noteItem(names[i]); const char* const name = tr(names[i]);
        const bool hov = ImGui::IsItemHovered(); ImGui::PushFont(fontMedium); const ImVec2 ts = ImGui::CalcTextSize(name, nullptr, true);
        dl->AddText({p.x+pad+i*cell+(cell-ts.x)/2, p.y+(h-ts.y)/2}, col(value == i ? text : hov ? text : textDim, a), name, visibleEnd(name)); ImGui::PopFont();
        ImGui::PopID();
    }
    ImGui::SetCursorScreenPos(p); ImGui::Dummy({w, h}); ImGui::PopID(); return changed;
}
static bool sliderCore(const char* label, float* t01, const char* valueText) {
    valueText = localDecimals(valueText);
    const char* const rawLabel = label; (void)rawLabel; label = tr(label);
    ImGui::PushID(label); const float w = ImGui::GetContentRegionAvail().x; const ImVec2 p = ImGui::GetCursorScreenPos(); const float a = ImGui::GetStyle().Alpha;
    const float lineH = ImGui::GetTextLineHeight(), h = lineH+px(20);
    ImGui::InvisibleButton("##sl", {w, h}); noteItem(rawLabel);
    const bool act = ImGui::IsItemActive(), hov = ImGui::IsItemHovered(); bool changed = false;
    const float trackY = p.y+lineH+px(11), x0 = p.x+px(7), x1 = p.x+w-px(7);
    if (act) { const float nv = std::clamp((ImGui::GetIO().MousePos.x-x0)/(x1-x0), 0.f, 1.f); if (nv != *t01) { *t01 = nv; changed = true; } }
    ImDrawList* dl = ImGui::GetWindowDrawList(); const float hl = animate(ImGui::GetItemID(), hov || act);
    if (*label != '#') dl->AddText({p.x, p.y}, col(textDim, a), label, visibleEnd(label));
    ImGui::PushFont(fontMedium); const ImVec2 vs = ImGui::CalcTextSize(valueText); dl->AddText({p.x+w-vs.x, p.y}, col(text, a), valueText); ImGui::PopFont();
    const float tx = x0+(x1-x0)**t01, tr = px(2.5f);
    dl->AddRectFilled({x0, trackY-tr}, {x1, trackY+tr}, col(hover, a), tr);
    dl->AddRectFilled({x0, trackY-tr}, {tx, trackY+tr}, col(sky, a), tr);
    const float r = px(7)+px(1.5f)*hl; if (hl > .01f) dl->AddCircleFilled({tx, trackY}, r+px(4)*hl, col(sky, .18f*hl*a));
    dl->AddCircleFilled({tx, trackY}, r, col(text, a));
    ImGui::PopID(); return changed;
}
bool slider(const char* label, float* v, float lo, float hi, const char* format, bool logarithmic) {
    float t = logarithmic ? std::log(std::max(*v, lo)/lo)/std::log(hi/lo) : (*v-lo)/(hi-lo); t = std::clamp(t, 0.f, 1.f);
    char buf[64]; snprintf(buf, sizeof buf, format, *v);
    if (!sliderCore(label, &t, buf)) return false;
    *v = logarithmic ? lo*std::pow(hi/lo, t) : lo+(hi-lo)*t; return true;
}
bool sliderInt(const char* label, int* v, int lo, int hi, const char* format) {
    float t = hi > lo ? std::clamp((float)(*v-lo)/(hi-lo), 0.f, 1.f) : 0; char buf[64]; snprintf(buf, sizeof buf, format, *v);
    if (!sliderCore(label, &t, buf)) return false;
    const int nv = lo+(int)std::lround(t*(hi-lo)); if (nv == *v) return false; *v = nv; return true;
}
bool toggle(const char* label, bool* v) {
    const char* const rawLabel = label; (void)rawLabel; label = tr(label);
    ImGui::PushID(label); const float h = ImGui::GetTextLineHeight()+px(4), w = h*1.75f, total = ImGui::GetContentRegionAvail().x; const ImVec2 p = ImGui::GetCursorScreenPos(); const float a = ImGui::GetStyle().Alpha;
    const bool r = ImGui::InvisibleButton("##tg", {total, h}); noteItem(rawLabel); if (r) *v = !*v;
    const bool hov = ImGui::IsItemHovered(); const float t = animate(ImGui::GetID("##pos"), *v, 16.f);
    ImDrawList* dl = ImGui::GetWindowDrawList(); const float x = p.x+total-w;
    dl->AddText({p.x, p.y+px(2)}, col(hov ? text : mix(textDim, text, .4f), a), label, visibleEnd(label));
    dl->AddRectFilled({x, p.y}, {x+w, p.y+h}, col(mix(ImVec4(.24f, .28f, .34f, 1), sky, t), a), h/2);
    dl->AddCircleFilled({x+h/2+(w-h)*t, p.y+h/2}, h/2-px(2.5f), col(text, a));
    ImGui::PopID(); return r;
}
bool combo(const char* label, int* current, const char* const* names, int n) {
    const char* const rawLabel = label; (void)rawLabel; label = tr(label);
    ImGui::PushID(label); bool changed = false;
    if (*label != '#') { ImGui::PushStyleColor(ImGuiCol_Text, textDim); ImGui::TextUnformatted(label, visibleEnd(label)); ImGui::PopStyleColor(); }
    ImGui::SetNextItemWidth(-1);
    const bool open = ImGui::BeginCombo("##c", tr(names[*current])); noteItem(rawLabel);
    if (open) { for (int i = 0; i < n; i++) { if (ImGui::Selectable(tr(names[i]), *current == i)) { changed = *current != i; *current = i; } noteItem(names[i]); } ImGui::EndCombo(); }
    ImGui::PopID(); return changed;
}
void beginCard(const char* id, const char* title, const char* icon) {
    ImGui::PushStyleColor(ImGuiCol_ChildBg, raised); ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, px(12)); ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {px(12), px(12)});
    ImGui::BeginChild(id, {-FLT_MIN, 0}, ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AlwaysUseWindowPadding, ImGuiWindowFlags_NoScrollbar);
    ImGui::PushStyleColor(ImGuiCol_FrameBg, bg); ImGui::PushStyleColor(ImGuiCol_Button, hover); ImGui::PushStyleColor(ImGuiCol_ButtonHovered, pressed);
    if (title) { title = tr(title);
        ImGui::PushFont(fontTitle);
        if (icon) { ImGui::PushStyleColor(ImGuiCol_Text, sky); ImGui::TextUnformatted(icon); ImGui::PopStyleColor(); ImGui::SameLine(0, px(8)); }
        ImGui::TextUnformatted(title); ImGui::PopFont(); ImGui::Dummy({0, px(2)});
    }
}
void endCard() { ImGui::PopStyleColor(3); ImGui::EndChild(); ImGui::PopStyleVar(2); ImGui::PopStyleColor(); ImGui::Dummy({0, px(2)}); }
bool collapsible(const char* label, bool defaultOpen) {
    ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0, 0, 0, 0)); ImGui::PushStyleColor(ImGuiCol_HeaderHovered, hover); ImGui::PushStyleColor(ImGuiCol_HeaderActive, pressed); ImGui::PushFont(fontMedium);
    const bool open = ImGui::CollapsingHeader(tr(label), defaultOpen ? ImGuiTreeNodeFlags_DefaultOpen : 0); noteItem(label); ImGui::PopFont(); ImGui::PopStyleColor(3); return open;
}
void hint(const char* t) { ImGui::PushStyleColor(ImGuiCol_Text, textDim); ImGui::TextWrapped("%s", tr(t)); ImGui::PopStyleColor(); }
void labelValue(const char* label, const char* value, const ImVec4& color) {
    label = tr(label); value = localDecimals(tr(value));
    ImGui::PushStyleColor(ImGuiCol_Text, textDim); ImGui::TextUnformatted(label); ImGui::PopStyleColor();
    ImGui::PushFont(fontMedium); const float w = ImGui::CalcTextSize(value).x; ImGui::SameLine(ImGui::GetContentRegionMax().x-w); ImGui::PushStyleColor(ImGuiCol_Text, color); ImGui::TextUnformatted(value); ImGui::PopStyleColor(); ImGui::PopFont();
}
static ImVec2 chipSize(const char* t, const char* icon) { ImVec2 s = ImGui::CalcTextSize(t); float w = s.x+px(20); if (icon) w += ImGui::CalcTextSize(icon).x+px(6); return {w, s.y+px(10)}; }
void chipAt(ImDrawList* dl, ImVec2 p, const char* t, const ImVec4& color, const char* icon) {
    t = localDecimals(tr(t));
    ImGui::PushFont(fontMedium); const ImVec2 sz = chipSize(t, icon); const float a = ImGui::GetStyle().Alpha;
    dl->AddRectFilled(p, {p.x+sz.x, p.y+sz.y}, col(ImVec4(.05f, .065f, .09f, .88f), a), sz.y/2);
    dl->AddRect(p, {p.x+sz.x, p.y+sz.y}, col(alpha(color, .32f), a), sz.y/2);
    float x = p.x+px(10); if (icon) { dl->AddText({x, p.y+px(5)}, col(color, a), icon); x += ImGui::CalcTextSize(icon).x+px(6); }
    dl->AddText({x, p.y+px(5)}, col(color, a), t); ImGui::PopFont();
}
void chip(const char* t, const ImVec4& color, const char* icon) { t = localDecimals(tr(t)); ImGui::PushFont(fontMedium); const ImVec2 sz = chipSize(t, icon); ImGui::PopFont(); const ImVec2 p = ImGui::GetCursorScreenPos(); ImGui::Dummy(sz); chipAt(ImGui::GetWindowDrawList(), p, t, color, icon); }
void spinner(ImDrawList* dl, ImVec2 c, float r, const ImVec4& color) {
    const float t = (float)ImGui::GetTime()*4.5f; const float a0 = t, a1 = t+3.14159f*1.35f;
    dl->PathArcTo(c, r, a0, a1, 32); dl->PathStroke(col(color), 0, px(2.5f));
}
void beginFloating(const char* id, ImVec2 pos, ImVec2 size) {
    ImGui::SetCursorScreenPos(pos); ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(.075f, .09f, .12f, .93f)); ImGui::PushStyleColor(ImGuiCol_Border, line);
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, size.y*.32f); ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1); ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {px(6), px(6)}); ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, {px(3), px(3)});
    ImGui::BeginChild(id, size, ImGuiChildFlags_Borders | ImGuiChildFlags_AlwaysUseWindowPadding, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
}
void endFloating() { ImGui::EndChild(); ImGui::PopStyleVar(4); ImGui::PopStyleColor(2); }

namespace { struct ToastItem { std::string text; Toast kind; double born; }; std::deque<ToastItem> toasts; }
void toast(const std::string& raw, Toast kind) {
    const std::string t = tr(raw.c_str());
    for (const ToastItem& i : toasts) if (i.text == t) return;
    toasts.push_back({t, kind, ImGui::GetTime()}); while (toasts.size() > 4) toasts.pop_front();
}
void drawToasts() {
    const double now = ImGui::GetTime(); ImGuiViewport* vp = ImGui::GetMainViewport(); ImDrawList* dl = ImGui::GetForegroundDrawList();
    const float width = px(340); float y = vp->Pos.y+vp->Size.y-px(18);
    for (int i = (int)toasts.size()-1; i >= 0; i--) {
        const ToastItem& t = toasts[i]; const double age = now-t.born; const double life = t.kind == Toast::Error ? 9 : 5;
        if (age > life) { toasts.erase(toasts.begin()+i); continue; }
        const float in = std::min(1.f, (float)age*7), out = std::clamp((float)(life-age)*3, 0.f, 1.f), a = in*out;
        const ImVec4& c = t.kind == Toast::Success ? green : t.kind == Toast::Warning ? amber : t.kind == Toast::Error ? danger : sky;
        const char* icon = t.kind == Toast::Success ? IC_CIRCLE_CHECK : t.kind == Toast::Warning ? IC_TRIANGLE_ALERT : t.kind == Toast::Error ? IC_CIRCLE_ALERT : IC_INFO;
        ImGui::PushFont(fontMedium); const ImVec2 ts = ImGui::CalcTextSize(t.text.c_str(), nullptr, false, width-px(52)); const float h = ts.y+px(24);
        const float x = vp->Pos.x+vp->Size.x-width-px(18)+(1-in)*px(24); y -= h;
        for (int k = 3; k >= 1; k--) dl->AddRectFilled({x-k*2.f, y-k*2.f+px(3)}, {x+width+k*2.f, y+h+k*2.f+px(3)}, IM_COL32(0, 0, 0, (int)(22*a)), px(12)+k*2.f);
        dl->AddRectFilled({x, y}, {x+width, y+h}, col(raised, a), px(12)); dl->AddRect({x, y}, {x+width, y+h}, col(alpha(c, .4f), a), px(12));
        dl->AddText({x+px(14), y+px(12)}, col(c, a), icon);
        dl->AddText(nullptr, 0, {x+px(40), y+px(12)}, col(text, a), t.text.c_str(), nullptr, width-px(52)); ImGui::PopFont();
        y -= px(8);
    }
}
void busyOverlay(ImVec2 pos, ImVec2 size, const char* title, float progress) {
    title = tr(title);
    ImDrawList* dl = ImGui::GetWindowDrawList(); dl->AddRectFilled(pos, {pos.x+size.x, pos.y+size.y}, col(ImVec4(.03f, .04f, .06f, .55f)), px(14));
    const float w = px(320), h = px(92); const ImVec2 p{pos.x+(size.x-w)/2, pos.y+(size.y-h)/2};
    dl->AddRectFilled(p, {p.x+w, p.y+h}, col(raised), px(14)); dl->AddRect(p, {p.x+w, p.y+h}, col(line), px(14));
    spinner(dl, {p.x+px(28), p.y+px(32)}, px(9), sky);
    ImGui::PushFont(fontMedium); dl->AddText({p.x+px(50), p.y+px(23)}, col(text), title); ImGui::PopFont();
    const float bx0 = p.x+px(16), bx1 = p.x+w-px(16), by = p.y+h-px(26); dl->AddRectFilled({bx0, by}, {bx1, by+px(5)}, col(hover), px(3));
    if (progress >= 0) dl->AddRectFilled({bx0, by}, {bx0+(bx1-bx0)*std::clamp(progress, 0.f, 1.f), by+px(5)}, col(sky), px(3));
    else { const float t = std::fmod((float)ImGui::GetTime()*.7f, 1.f), a = bx0+(bx1-bx0)*std::max(0.f, t*1.4f-.4f), b = bx0+(bx1-bx0)*std::min(1.f, t*1.4f); dl->AddRectFilled({a, by}, {b, by+px(5)}, col(sky), px(3)); }
}
bool imageTile(unsigned texture, const char* letter, const ImVec4& color, const char* label, ImVec2 size, bool dim) {
    const ImVec2 p = ImGui::GetCursorScreenPos(), q{p.x+size.x, p.y+size.y}; ImGui::PushID(label);
    const bool pressedNow = ImGui::InvisibleButton("##tile", size), hov = ImGui::IsItemHovered(), held = ImGui::IsItemActive(); noteItem(label); ImGui::PopID(); label = tr(label);
    ImDrawList* dl = ImGui::GetWindowDrawList(); const float r = px(10), alphaAll = dim ? .45f : 1.f;
    dl->AddRectFilled(p, q, col(held ? pressed : hov ? hover : raised), r);
    dl->AddRect(p, q, col(hov ? lineStrong : line), r);
    const ImVec2 ls = fontBody->CalcTextSizeA(ImGui::GetFontSize()*.84f, FLT_MAX, 0, label); const float box = std::min(size.x-px(24), size.y-ls.y-px(18));
    const ImVec2 a{p.x+(size.x-box)/2, p.y+px(7)}, b{a.x+box, a.y+box};
    if (texture) dl->AddImageRounded((ImTextureID)(intptr_t)texture, a, b, {0, 0}, {1, 1}, IM_COL32(255, 255, 255, (int)(255*alphaAll)), px(6));
    else {
        dl->AddRectFilled(a, b, col(color, alphaAll), px(8));
        const float fs = box*.62f; const ImVec2 t = fontTitle->CalcTextSizeA(fs, FLT_MAX, 0, letter); dl->AddText(fontTitle, fs, {a.x+(box-t.x)/2, a.y+(box-t.y)/2}, IM_COL32(255, 255, 255, (int)(255*alphaAll)), letter);
    }
    dl->AddText(fontBody, ImGui::GetFontSize()*.84f, {p.x+(size.x-ls.x)/2, q.y-ls.y-px(5)}, col(dim ? textFaint : textDim), label);
    return pressedNow;
}
}
