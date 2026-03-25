module;

#include <bit>

module board;

namespace board {
    bool operator==(Board first, Board second) {
        return first.white == second.white && first.black == second.black;
    }

    ChangeType validateChange(Board current, Board next, bool whitesTurn) {
        if (current == EMPTY_BOARD && next == STARTING_BOARD) {
            return ChangeType::VALID;
        }

        int count = std::popcount(whitesTurn ? current.white ^ next.white : current.black ^ next.black);

        if (count != 2 && count != 4) {
            return ChangeType::INVALID;
        }

        if (count == 4) {
            // check castling
        } else {
            // generate move - promotion of piece???
        }

        // compare to legal moves set
        // maybe validate captures on other board??
    }
}
