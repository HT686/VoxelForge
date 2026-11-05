#include "i18n.hpp"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <unordered_map>

namespace {
struct Row { const char* text[4]; };
const Row kRows[] = {
#include "translations.inc"
};
constexpr int kRowCount = sizeof kRows/sizeof *kRows;
std::unordered_map<std::string, const Row*>& rowsByGerman() {
    static std::unordered_map<std::string, const Row*> map = [] { std::unordered_map<std::string, const Row*> m; m.reserve(kRowCount*2); for (const Row& r : kRows) m.emplace(r.text[0], &r); return m; }();
    return map;
}
Language systemLanguage() {
    // The usual order of precedence. LANGUAGE may list several, the first one that we know counts.
    if (const char* list = getenv("LANGUAGE")) {
        std::string all = list; size_t a = 0;
        while (a < all.size()) { size_t b = std::min(all.find(':', a), all.size()); const std::string one = all.substr(a, b-a); if (one.size() >= 2) { const char* n = one.c_str(); if (!strncmp(n, "de", 2) || !strncmp(n, "fr", 2) || !strncmp(n, "es", 2) || !strncmp(n, "en", 2)) return languageFrom(n); } a = b+1; }
    }
    for (const char* name : {"LC_ALL", "LC_MESSAGES", "LANG"}) if (const char* v = getenv(name)) if (*v) return languageFrom(v);
    return Language::English;
}
Language& current() { static Language l = [] { if (const char* forced = getenv("VOXELFORGE_LANG")) return languageFrom(forced); return systemLanguage(); }(); return l; }
std::mutex gMutex;
std::unordered_map<std::string, std::string> gCache[4];
}
Language language() { return current(); }
Language languageFrom(const char* n) {
    if (!n || strlen(n) < 2) return Language::English;
    // Only the language counts, not the country: de_AT, fr_CA and es_MX are served too.
    if (!strncmp(n, "de", 2) && (n[2] == 0 || n[2] == '_' || n[2] == '.' || n[2] == '@')) return Language::German;
    if (!strncmp(n, "fr", 2) && (n[2] == 0 || n[2] == '_' || n[2] == '.' || n[2] == '@')) return Language::French;
    if (!strncmp(n, "es", 2) && (n[2] == 0 || n[2] == '_' || n[2] == '.' || n[2] == '@')) return Language::Spanish;
    return Language::English;
}
void setLanguage(Language l) { current() = l; }
int translationCount() { return kRowCount; }
const char* translationText(int line, int lang) { return kRows[line].text[lang]; }
const char* tr(const char* german) {
    if (!german || !*german) return german;
    const Language lang = language(); if (lang == Language::German) return german;
    std::lock_guard<std::mutex> g(gMutex);
    auto& cache = gCache[(int)lang]; const auto seen = cache.find(german);
    if (seen != cache.end()) return seen->second.c_str();
    // A leading icon (private use characters) with its spaces and a trailing ##id stay as they are.
    const char* begin = german;
    while ((unsigned char)begin[0] == 0xEE && begin[1] && begin[2]) { begin += 3; while (*begin == ' ') begin++; }
    const char* hash = strstr(begin, "##"); const std::string key = hash ? std::string(begin, hash) : std::string(begin);
    std::string out = german;
    const auto row = rowsByGerman().find(key);
    if (row != rowsByGerman().end()) out = std::string(german, begin)+row->second->text[(int)lang]+(hash ? hash : "");
    else if (const char* log = getenv("VOXELFORGE_I18N_MISSING")) if (FILE* f = fopen(log, "a")) { fprintf(f, "%s\n", key.c_str()); fclose(f); }   // for checking that nothing was left out
    return cache.emplace(german, std::move(out)).first->second.c_str();
}
std::string trf(const char* german, const std::string& value) {
    std::string t = tr(german); const size_t at = t.find("%s"); if (at != std::string::npos) t.replace(at, 2, value); return t;
}
const char* localDecimals(const char* text) {
    if (!text || language() == Language::English) return text;
    // A decimal point stands between digits and has one or two digits after it. Thousands always have three.
    static thread_local std::string ring[8]; static thread_local int next = 0; std::string& s = ring[next]; next = (next+1)%8; s = text;
    const auto digit = [&](size_t p) { return p < s.size() && s[p] >= '0' && s[p] <= '9'; };
    for (size_t p = 1; p+1 < s.size(); p++) if (s[p] == '.' && digit(p-1) && digit(p+1) && (!digit(p+2) || !digit(p+3))) { size_t e = p+1; while (digit(e)) e++; if (e-p-1 <= 2) s[p] = ','; }
    return s.c_str();
}

