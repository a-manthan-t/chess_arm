module;

#include <string>

export module streamer;

import arm;
import camera;

export namespace streamer {
    [[noreturn]] void stream(const std::string& url, const std::string& token, unsigned int fps, camera::Camera* cam);
}
