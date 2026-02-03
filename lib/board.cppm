module;

#include <cstdint>

export module board;

export namespace board {
    struct Board {
        // bitboard representation, so checking differences between two boards
        // becomes much simpler and efficient - a single xor operation (move comment elsewhere).
        uint64_t white {}, black {};

        Board operator^(Board other) const;
    };

    enum class ChangeType { NONE, VALID, INVALID };
    ChangeType validateChange(Board board, bool white);
}
