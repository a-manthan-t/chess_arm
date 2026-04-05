module;

#include <string>
#include <unistd.h>

export module engine;

export namespace engine {
    class Engine {
        static constexpr size_t OFFSET { std::string("bestmove ").length() };
        static constexpr std::string REQUEST_SEARCH { "go movetime 1000\n" };

        int child[2] {}, parent[2] {}; // Pipes.
        pid_t childPid;
        std::string moveHistory { "position startpos moves " };

        public:
            Engine(const char* command);
            ~Engine();

            std::string getMove(const std::string& lastMove);
    };
}
