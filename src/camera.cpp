module;

#include <mutex>
#include <ranges>

#include <opencv2/opencv.hpp>

// Handles taking and processing camera input.
module camera;

import arm;
import board;

namespace camera {
    // Detect the grid on a chess board set to the starting position.
    bool Camera::configure() {
        if (!camera.read(raw)) {
            std::println(stderr, "Configuration failed because the camera could not capture an image.");
            return false;
        }

        // Extract straight lines from the image.
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

        float squareWidth { points[2].x - points[1].x }; // Since at most the first square is invalid but the second is fine.
        cv::Point2f squareOffset { squareWidth / 8, squareWidth / 8 };
        cv::Size2f squareSize { squareWidth * 0.75f, squareWidth * 0.75f };

        int i {}, j { rowCount > 9 }, k { firstCol };

        while (i < squares.size()) {
            squares[i++] = { points[j * colCount + k] + squareOffset, squareSize };

            if (++k == 8 + firstCol) {
                k = firstCol;
                j++;
            }
        }

        board::Board startBoard { processRaw() };
        if (board::validateChange(currentBoard, startBoard, true) == board::ChangeType::VALID) {
            currentBoard = startBoard;
        } else {
            std::println(stderr, "Configuration failed because the board is not in the starting position.");
            return false;
        }

        return true;
    }

    // Take an image and detect the squares taken up by white
    // and black pieces, returning a bitboard representation.
    board::Board Camera::processRaw() {
        if (!camera.read(raw)) {
            return board::EMPTY_BOARD;
        }

        frame = raw.clone();

        cv::Mat hsv;
        cv::cvtColor(raw, hsv, cv::COLOR_BGR2HSV);
        cv::Scalar mean {}, std {};
        board::Board newBoard {};

        for (int i {}; i < squares.size(); ++i) {
            cv::meanStdDev(hsv(squares[whitesPerspective ? i : squares.size() - i - 1]), mean, std);

            newBoard.white <<= 1;
            newBoard.black <<= 1;

            int colour {};

            if (std.val[1] > 15 && std.val[2] > 15) {
                if (true) {
                    newBoard.white++;
                    colour = 1;
                } else {
                    newBoard.black++;
                    colour = -1;
                }
            }

            rectangle(frame, squares[i-1], !colour ? GREEN : colour == 1 ? WHITE : BLACK, 2);
        }

        return newBoard;
    }

    void Camera::generateCheckpoints() {

    }

    // For streaming purposes.
    void Camera::encodeFrame() {
        if (!frame.empty()) {
            std::lock_guard lock { cameraMutex }; // Lock so streamer doesn't read a partially ready image.
            cv::imencode(".jpg", frame, buffer);
        }
    }

    [[noreturn]] void Camera::loop() {
        // configure??
        while (true) {
            // captire first
            processRaw();
            generateCheckpoints();
            encodeFrame();
        }
    }
}
