#pragma once
// Every file the program opens or saves is chosen in a dialog. The desktop's own one if it has a helper
// program for it (kdialog, zenity), otherwise a simple one drawn here. It never blocks the window.
#include <functional>
#include <string>
#include <vector>

namespace filedialog {
enum class Kind { Open, Save, Folder };
struct Filter { std::string name; std::vector<std::string> extensions; };   // extensions without the dot
// done is called on the UI thread with the path. It is not called if the user cancels.
void request(Kind kind, const std::string& title, const std::string& start, const std::vector<Filter>& filters, std::function<void(const std::string&)> done);
void frame();     // once per frame: dims the window while a dialog is open, delivers the answer
bool active();
}
