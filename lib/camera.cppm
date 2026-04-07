module;

#include <mutex>
#include <vector>

#include <opencv2/videoio.hpp>

export module camera;

import arm;
import chess_library;
import engine;
import path;

export namespace camera {
    constexpr std::pair
        STARTING_BOARD { 65535ULL, 18446462598732840960ULL },
        FLIPPED_STARTING_BOARD { 18446462598732840960ULL, 65535ULL };
    const cv::Scalar BLACK { 0, 0, 0 }, WHITE { 255, 255, 255 }, BLUE { 255, 255, 0 }, GREEN { 120, 220, 80 };
    typedef uint16_t MoveType;

    enum class ChangeType { VALID, INVALID, NONE };
    std::string pieceToString(chess::Piece type);

    class Camera {
        cv::VideoCapture camera { PIPELINE, cv::CAP_ANY };
        cv::Mat raw, frame;

        std::array<cv::Rect2f, 64> squares {};
        chess::Board board {};
        arm::Arm* robot {};
        engine::Engine chessEngine;
        float squareWidth;
        bool whitesPerspective {};

        std::pair<uint64_t, uint64_t> processRaw();
        std::string checkChange(const std::pair<uint64_t, uint64_t>& newBitboard);
        void generateCheckpoints(const std::string &move, MoveType moveType) const;
        void encodeFrame();

        public:
            Camera(arm::Arm* robot, const engine::Engine& chessEngine, float squareWidth)
                : robot(robot), chessEngine(chessEngine), squareWidth(squareWidth) {}

            std::mutex cameraMutex;
            std::vector<unsigned char> buffer;

            bool configure();
            [[noreturn]] void loop();
    };
}
