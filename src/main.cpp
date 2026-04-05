#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <fstream>
#include <print>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>

import quaternion;
import streamer;
import camera;
import arm;
import chess_library;
import engine;

#ifdef TESTING
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "../lib/doctest.h"
#else
int main(int argc, char* argv[]) {
    using namespace cv; // stop this
    Mat raw { imread("test_images/move1.jpg") }, write { imread("test_images/move1.jpg") };
    cv::Mat blur, edges;
    std::vector<cv::Vec2f> lines; // Lines are in polar form.

    // Much of these numbers are somewhat arbitrary and inexact -
    // the algorithm is robust enough for it to not matter.
    cv::cvtColor(raw, blur, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(blur, blur, { 5, 5 }, 0);
    cv::Canny(blur, edges, 50, 150);
    cv::HoughLines(edges, lines, 1, CV_PI / 180, 100, 50, 10);

    // Keep only vertical and horizontal lines. Distinction of horizontal
    // and vertical improves intersection finding efficiency.
    std::vector<cv::Vec2f> horizontalLines, verticalLines;
    constexpr double ERROR_ANGLE { CV_PI / 180 * 2 };

    for (cv::Vec2f line : lines) {
        if (CV_PI / 2 - ERROR_ANGLE < line[1] && line[1] < CV_PI / 2 + ERROR_ANGLE ) {
            horizontalLines.push_back(line);
        } else if (line[1] < ERROR_ANGLE || line[1] > CV_PI - ERROR_ANGLE) {
            verticalLines.push_back(line);
        }
    }

    // Find unique intersection points of the lines.
    std::vector<cv::Point2f> points;

    for (cv::Vec2f f : horizontalLines) {
        for (cv::Vec2f g : verticalLines) {
            // No need to normalise since lines are near perpendicular, so scale will be 1 or -1.
            bool duplicate {};
            float
                x { std::abs(f[0] * std::sin(g[1]) - g[0] * std::sin(f[1])) },
                y { std::abs(g[0] * std::cos(f[1]) - f[0] * std::cos(g[1])) };

            // Check for duplicate points.
            for (cv::Point2f p : points) {
                float dx { p.x - x }, dy { p.y - y };
                if (dx * dx + dy * dy <= 25 * 25) { // Each point should be at least 25 pixels away from any other.
                    duplicate = true;
                    break;
                }
            }

            if (!duplicate) {
                points.emplace_back(x, y);
            }
        }
    }

    // Order all the points so that the first is in the top left, second is to the right of it, etc.
    std::ranges::sort(points, [](cv::Point2f p1, cv::Point2f p2) { return p1.x + p1.y * 100 < p2.x + p2.y * 100;});

    // Some chessboards have an extra border around the squares, which may get mistaken
    // as part of the main board, so we need to get rid of these extra points before
    // generating our squares. If there are more than 9 rows/columns of points we
    // can ignore the first.

    int rowCount { 1 }, colCount { 1 };
    while (points[colCount - 1].x < points[colCount].x) {
        colCount++;
    }
    while (points[(rowCount - 1) * colCount].y < points[rowCount * colCount].y) {
        rowCount++;
    }

    bool firstCol { colCount > 9 };
    int i {}, j { 7 + (rowCount > 9) }, k { firstCol };

    std::array<cv::Rect2f, 64> squares {};

    while (i < squares.size()) {
        Point2f tl { points[j * colCount + k] }, br { points[(j + 1) * colCount + k + 1] }, vec { (br - tl)  * 0.2 };
        squares[true ? i++ : 63 - i++] = { tl + vec, br - vec };

        if (++k == 8 + firstCol) {
            k = firstCol;
            j--;
        }
    }

    chess::Board board {};
    std::pair<uint64_t, uint64_t> bitboard {};

    [&] {
        cv::Mat hsv;
        cv::cvtColor(raw, hsv, cv::COLOR_BGR2HSV);
        cv::Scalar mean {}, std {};

        for (int rank { 7 }; rank >= 0; --rank) {
            for (int file { 7 }; file >= 0; --file) {
                int idx { rank * 8 + file };

                cv::meanStdDev(hsv(squares[idx]), mean, std);

                bitboard.first <<= 1;
                bitboard.second <<= 1;

                int colour {};
                constexpr double OCCUPIED_THRESHOLD { 25 }, WB_THRESHOLD { 85 };

                if (std.val[2] > OCCUPIED_THRESHOLD) {
                    if ((mean.val[0] + mean.val[1] + mean.val[2]) / 3 > WB_THRESHOLD) {
                        bitboard.first++;
                        colour = 1;
                    } else {
                        bitboard.second++;
                        colour = -1;
                    }
                }

                rectangle(write, squares[idx], colour == 0 ? camera::BLUE : colour == 1 ? camera::WHITE : camera::BLACK, 2);
                putText(
                    write,
                    camera::pieceToString(board.at(chess::Square(idx))),
                    squares[idx].tl() * 0.67 + squares[idx].br() * 0.33,
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    camera::GREEN
                );
            }
        }
    }();

    std::string x = [&] -> std::string {
        if (board.us(chess::Color::WHITE).getBits() == bitboard.first && board.us(chess::Color::BLACK).getBits() == bitboard.second) {
            return "N"; // No change
        }

        chess::Movelist moves;
        chess::movegen::legalmoves(moves, board);

        for (const chess::Move move : moves) {
            // This line implements the auto queen rule (simplifies user interface and
            // makes vision more robust with little loss of game functionality, since
            // promotions are almost always made to a queen).
            if (move.typeOf() == chess::Move::PROMOTION && move.promotionType() != chess::PieceType::QUEEN) {
                continue;
            }

            board.makeMove(move);

            if (board.us(chess::Color::WHITE).getBits() == bitboard.first && board.us(chess::Color::BLACK).getBits() == bitboard.second) {
                return chess::pgn::uci::moveToUci(move); // Return the move that was made.
            }

            board.unmakeMove(move);
        }

        return "I"; // Invalid change
    }();

    std::println("{}", x);

    imwrite("processed.jpg", write);

    return 1;

    using namespace arm;
    using namespace camera;
    using namespace engine;

    if (argc < 2) {
        std::println(stderr, "Please provide a configuration file.");
        return 1;
    }

    if (argc < 3) {
        std::println(stderr, "Please provide a chess engine program.");
        return 1;
    }

    std::ifstream config { argv[1] };

    if (!config) {
        std::println(stderr, "Could not open configuration file.");
        return 1;
    }

    std::string line, token;

    /* Get stream configuration. */

    if (!std::getline(config, line)) {
        std::println(stderr, "Unexpected configuration file end.");
        return 1;
    }

    std::istringstream lineStream { line };
    std::vector<std::string> streamConfig;

    while (std::getline(lineStream, token, ';')) {
        streamConfig.push_back(token);
    }

    /* Get the number of wrist joints in the arm. */

    if (!std::getline(config, line)) {
        std::println(stderr, "Unexpected configuration file end.");
        return 1;
    }

    size_t wristSize;
    auto [_, err] = std::from_chars(line.data(), line.data() + line.size(), wristSize);

    if (err != std::errc {}) {
        std::println(stderr, "Could not parse wrist size.");
        return 1;
    }

    /* Get the descriptions of each joint in the arm. */

    std::vector<Joint> joints;

    while (std::getline(config, line)) {
        if (line.size() < 5) { // Should at least be A;_;B.
            std::println(stderr, "Could not parse joints.");
            return 1;
        }

        float segmentLength;
        auto [_, err] = std::from_chars(line.data() + 2, line.data() + line.size() - 2, segmentLength);

        if (err != std::errc {}) {
            std::println(stderr, "Could not parse joints.");
            return 1;
        }

        Axis armAxis { line.front() == 'X' ? Axis::X : line.front() == 'Y' ? Axis::Y : Axis::Z };
        Axis rotAxis { line.back() == 'X' ? Axis::X : line.back() == 'Y' ? Axis::Y : Axis::Z };
        joints.emplace_back(armAxis, segmentLength, rotAxis);
    }

    if (joints.size() < wristSize) {
        std::println(stderr, "Wrist size is greater than number of joints.");
        return 1;
    }

    /* Configure and start the robot, vision, and streaming threads. */

    Arm robot { joints, wristSize };
    Camera cam { &robot, { argv[2] } }; // TODO restart camera on game end to reset, and have a start game mechanism.

    std::thread arm { &Arm::follow, &robot };
    std::thread vision { &Camera::loop, &cam };

    if (streamConfig.size() == 3) {
        unsigned int fps;
        auto [_, err] = std::from_chars(streamConfig[2].data(), streamConfig[2].data() + streamConfig[2].size(), fps);

        if (err == std::errc {}) {
            std::thread stream { &streamer::stream, streamConfig[0], streamConfig[1], fps, &cam };
            std::println("Configured streaming to {} at {} FPS.", streamConfig[0], fps);
            stream.join();
        } else {
            std::println(stderr, "Could not parse FPS.");
            return 1;
        }
    } else {
        std::println(stderr, "Could not parse stream config.");
        return 1;
    }

    arm.join();
    vision.join();

    return 0;
}
#endif
