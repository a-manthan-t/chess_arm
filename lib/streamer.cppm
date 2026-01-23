module;

#include <string>

export module streamer;

import arm;

export namespace streamer {
    [[noreturn]] void stream(const std::string& url, const std::string& token, unsigned int fps, const arm::Arm* robot);
}
