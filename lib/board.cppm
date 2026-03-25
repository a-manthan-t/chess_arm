module;

#include <cstdint>

export module board;

export namespace board {
    struct Board {
        uint64_t white {}, black {};

        Board() = default;
        constexpr Board(uint64_t white, uint64_t black) : white(white), black(black) {}
    };

    // These correspond to integers with the last 16 bits and first 16 bits set,
    // representing the positions of the pieces at the start of a chess game.
    constexpr Board STARTING_BOARD { 65535ULL, 18446462598732840960ULL };
    constexpr Board EMPTY_BOARD { 0ULL, 0ULL };

    enum class ChangeType { NONE, VALID, INVALID };

    bool operator==(Board first, Board second);
    ChangeType validateChange(Board current, Board next, bool whitesTurn);
}
