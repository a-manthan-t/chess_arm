module;

#include <bit>

module board;

namespace board {
    Board Board::operator^(Board other) const {
        return { white ^ other.white, black ^ other.black };
    }

    ChangeType validateChange(Board board, bool white) {
        int changes = std::popcount(white ? board.white : board.black);

        if (changes != 2 && changes != 4) {
            return ChangeType::INVALID;
        }

        if (changes == 4) {
            // check castling
        } else {
            // generate move - promotion of piece???
        }

        // compare to legal moves set
    }
}
