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

using namespace quaternion;

#ifdef TESTING
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "../lib/doctest.h"
#else
int main(int argc, char* argv[]) {
    using namespace cv; // stop this
    Mat raw { imread("start.jpg") }, write { imread("start.jpg") };
    // Extract straight lines from the image.
        cv::Mat blur, edges, hsv;
        std::vector<cv::Vec2f> lines; // Lines are in polar form.

        // Much of these numbers are somewhat arbitrary and inexact -
        // the algorithm is robust enough for it to not matter.
        cv::cvtColor(raw, blur, cv::COLOR_BGR2GRAY);
        cv::cvtColor(raw, hsv, cv::COLOR_BGR2HSV);
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
        // can ignore the first

        int rowCount { 1 }, colCount { 1 };
        while (points[colCount - 1].x < points[colCount].x) {
            colCount++;
        }
        while (points[(rowCount - 1) * colCount].y < points[rowCount * colCount].y) {
            rowCount++;
        }
        bool firstCol { colCount > 9 };

        float squareWidth { points[2].x - points[1].x }; // Since at most the first square is invalid but the second is fine.
        cv::Point2f squareOffset { squareWidth / 8, squareWidth / 8 };
        cv::Size2f squareSize { squareWidth * 0.75f, squareWidth * 0.75f };

        int i {}, j { rowCount > 9 }, k { firstCol };
        std::array<cv::Rect2f, 64> squares {};
        write = raw.clone();
        while (i < squares.size()) {
            squares[i++] = { points[j * colCount + k] + squareOffset, squareSize };

            cv::Scalar mean {}, std {};
            meanStdDev(hsv(squares[i-1]), mean, std);
            rectangle(write, squares[i-1], { (std.val[1] > 15 && std.val[2] > 15) * 255., 255, 0 }, 2);

            if (++k == 8 + firstCol) {
                k = firstCol;
                j++;
            }
        }
    // putText(write, std::format("{:.2f} {:.2f}", mean.val[1], mean.val[2]), squares[i-1].tl() + squareOffset, cv::FONT_HERSHEY_SIMPLEX, 0.2, {0, 255, 255});

    imwrite("post_lines.jpg", write);

    // camera::Board chessBoard {};
    // chess::Board board {};
    //
    // chess::Movelist moves;
    // chess::movegen::legalmoves(moves, board);
    // for (chess::Move move : moves) {
    //     // std::println("{}", chess::uci::moveToUci(move));
    // }
    //
    // // Render
    // // also efficiency?!
    // for (int y { borderedBoard }; y < breakout; ++y) {
    //     int row { y * rowIncrement }, nextRow { row + rowIncrement };
    //     for (int x { borderedBoard }; x < breakout; ++x) {
    //         if (x + 1 != breakout && y + 1 != breakout) {
    //             Point
    //                 tl { points[x + row] },
    //                 br { points[x + 1 + nextRow] },
    //                 vec { br - tl };
    //
    //             Rect bounds { tl + vec/8, br - vec/8 };
    //             Mat square { gray(bounds) }, shapes;
    //             GaussianBlur(square, square, { 5, 5 }, 0);
    //             Canny(square, shapes, 100, 150);
    //
    //             std::vector<std::vector<Point>> contours;
    //             findContours(shapes, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
    //
    //             double length {};
    //             for (std::vector<Point>& contour : contours) {
    //                 length += arcLength(contour, false);
    //
    //                 for (Point& p : contour) {
    //                     p.x += bounds.tl().x;
    //                     p.y += bounds.tl().y;
    //                 }
    //             }
    //
    //             chessBoard.white <<= 1;
    //             chessBoard.black <<= 1;
    //
    //             std::array<double, 4> mu {}, sigma2 {};
    //             meanStdDev(hsv(bounds), mu, sigma2);
    //
    //             if (sigma2[1] > 10 && length > 50) {
    //                 std::println("{} {}", mu, sigma2);
    //                 bool white { mu[1] > 70 };
    //
    //                 if (white) {
    //                     chessBoard.white += 1;
    //                 } else {
    //                     chessBoard.black += 1;
    //                 }
    //
    //                 rectangle(
    //                     img,
    //                     bounds,
    //                     white ? Scalar { 255, 255, 255 } : Scalar { 0, 0, 0 },
    //                     2
    //                 );
    //
    //                 drawContours(img, contours, -1, {0, 0, 255});
    //
    //                 putText(
    //                     img,
    //                     format("l: %.2f, s: %.2f", length, mu[1]),
    //                     { tl.x, br.y - vec.y / 3 },
    //                     FONT_HERSHEY_SIMPLEX,
    //                     0.3,
    //                     { 0, 255, 255 },
    //                     1
    //                 );
    //             }
    //         }
    //     }
    // }
    //
    // // Write result to file
    // imwrite("post_lines.jpg", img);

    // Check past frame and current one, classify squares based on obscured and empty (check colour consistency?)
    // and compare between frames - will need some way of checking in case of promotions - perhaps keep track of
    // removed and placed pieces - require no cheating ofc.

    return 1;

    using namespace arm;
    using namespace camera;

    if (argc < 2) {
        std::println(stderr, "Please provide a configuration file.");
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
    Camera cam { &robot, true }; // TODO input perspective

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
