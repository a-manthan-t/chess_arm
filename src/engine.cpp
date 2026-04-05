module;

#include <print>
#include <string>
#include <unistd.h>

module engine;

namespace engine {
    // Open connection to engine upon construction.
    Engine::Engine(const char* command)  {
        if (pipe(child) == -1 || pipe(parent) == -1) {
            std::println(stderr, "Could not start engine.");
        }

        childPid = fork();

        if (childPid == 0) {  // Child process.
            close(child[0]);  // We don't want the child to be able to read from itself.
            close(parent[1]); // Or write to where the parent is supposed to write.

            dup2(child[1], STDOUT_FILENO);  // Make the child write to its end of the child pipe.
            dup2(parent[0], STDIN_FILENO);  // And read input from its end of the parent pipe.

            execlp(command, "engine", nullptr);

            std::println("An error occurred whilst running the engine.");
            exit(1);          // On fail.
        }

        close(child[1]);    // Don't want the parent writing to where the child should.
        close(parent[0]);   // Or reading from itself.
    }

    // Cleanup engine connection upon destruction of object.
    Engine::~Engine() {
        write(parent[1], "quit\n", 5); // Tell engine to quit.

        close(child[0]);               // End connection.
        close(parent[1]);

        waitpid(childPid, nullptr, 0); // Discard the child process.
    }

    // Request best move in position from engine.
    std::string Engine::getMove(const std::string& lastMove) {
        moveHistory += lastMove + ' ';

        // Tell engine to take 1 second to calculate a move based on current position.
        write(parent[1], (moveHistory + '\n').c_str(), moveHistory.length() + 1);
        write(parent[1], REQUEST_SEARCH.c_str(), REQUEST_SEARCH.length());

        std::string buffer {}, move {};
        buffer.resize(128);

        // Extract move from engine output.
        while (read(child[0], buffer.data(), buffer.size()) > 0) {
            if (size_t index { buffer.find("bestmove") }; index != std::string::npos) {
                size_t moveStart { index + OFFSET };
                move = buffer.substr(moveStart, buffer.find(' ', moveStart) - moveStart);
                break;
            }
        }

        switch (move.back()) {
            case 'r':
            case 'n':
            case 'b': move.back() = 'q';
            default:  break;
        }

        moveHistory += move + ' ';
        return move;
    }
}
