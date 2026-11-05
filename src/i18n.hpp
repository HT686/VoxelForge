#pragma once
// The program speaks the language of the system: German, English, French or Spanish, otherwise English.
// German text in the code is the key. All languages are in translations.inc, one line per text.
#include <string>

enum class Language { German, English, French, Spanish };
Language language();
Language languageFrom(const char* localeName);   // "fr_CH.UTF-8" gives French, anything unknown English
void setLanguage(Language l);
// The text in the current language, with its icon and ## part kept. Text it does not know comes back as it is.
const char* tr(const char* german);
// For a sentence around one value: the %s in the translated text is replaced.
std::string trf(const char* german, const std::string& value);
// For the tests: number of lines, and the line's texts in all four languages.
int translationCount();
const char* translationText(int line, int language);
