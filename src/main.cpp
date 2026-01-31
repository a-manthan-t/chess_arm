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

using namespace quaternion;

#ifdef TESTING
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "../lib/doctest.h"
#else
int main(int argc, char* argv[]) {
    using namespace cv; // stop this
    Mat img { imread("top_down_board.jpg") };

    // Preprocess
    Mat gray, blurImg, edges;
    cvtColor(img, gray, COLOR_BGR2GRAY);
    // threshold(gray, gray, 127, 255, THRESH_BINARY);
    imwrite("output_bw.jpg", gray);

    GaussianBlur(gray, blurImg, Size(5, 5), 0);
    Canny(blurImg, edges, 50, 150);

    // Line detection
    std::vector<Vec2f> lines;

    HoughLines(
        edges,
        lines,
        1,
        CV_PI / 180,
        100,
        50,
        10
    );

    std::vector<Vec2f> horizontalLines, verticalLines; // separation reduces complexity of intersection finding

    // COMMENT and move to camera!!!
    for (const auto& l : lines) {
        // rename stuff, deal with magic numbers
        constexpr double angleh = CV_PI / 180 * 2;
        constexpr double anglev1 = CV_PI / 180 * 2;
        constexpr double anglev2 = CV_PI / 180 * 178;

        bool horizontal = std::abs(l[1] - CV_PI/2) < angleh;
        bool vertical = l[1] < anglev1 || l[1] > anglev2;

        if (horizontal) {
            horizontalLines.push_back(l);
        } else if (vertical) {
            verticalLines.push_back(l);
        }
    }

    std::vector<Point> points;

    for (Vec2f f : verticalLines ) {
        double rho1 { f[0] }, theta1 { f[1] };

        for (Vec2f g : horizontalLines) {
            double rho2 { g[0] }, theta2 { g[1] };
            double det = std::cos(theta1)*std::sin(theta2) - std::sin(theta1)*std::cos(theta2);

            int
                x { static_cast<int>((rho1 * std::sin(theta2) - rho2 * std::sin(theta1)) / det) },
                y { static_cast<int>((rho2 * std::cos(theta1) - rho1 * std::cos(theta2)) / det) };

            bool duplicate {};
            for (Point p : points) {
                int dx { p.x - x }, dy { p.y - y };
                if (dx * dx + dy * dy <= 25 * 25) {
                    duplicate = true;
                    break;
                }
            }

            if (!duplicate) {
                points.emplace_back(x, y);
            }
        }
    }

    if (points.size() < 40) {
        // make proper error
        std::println(stderr, "The camera could not be calibrated properly, please adjust and try again.");
        return 1;
    }

    bool bordeedBoard { true };
    int breakout { 9 + bordeedBoard }, rowIncrement { bordeedBoard ? 11 : 9 }; // fewer magic numbers!

    std::ranges::sort(points, [](Point p1, Point p2) { return p1.y < p2.y; });
    for (int y { bordeedBoard }; y < breakout; ++y) {
        int row { y * rowIncrement }, nextRow { row + rowIncrement };
        std::sort(points.begin() + row, points.begin() + nextRow, [](Point p1, Point p2) { return p1.x < p2.x; });
    }

    std::array<std::array<std::array<Point, 4>, 8>, 8> board;

    // Render
    for (int y { bordeedBoard }; y < breakout; ++y) {
        int row { y * rowIncrement }, nextRow { row + rowIncrement };
        for (int x { bordeedBoard }; x < breakout; ++x) {
            if (x + 1 != breakout && y + 1 != breakout) {
                std::array quad = { points[x + row], points[x + 1 + row], points[x + 1 + nextRow], points[x + nextRow] };
                polylines(img, quad, true, { 0, 255, 0 }, 3);
                board[x - bordeedBoard][breakout - y - 2] = quad;
            }

            circle(img, points[x + row], 5, { 0, 0, 255 }, -1);
        }
    }

    polylines(img, board[4][3], true, { 255, 255, 0 }, 10);

    // Write result to file
    imwrite("output_lines.jpg", img);

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

    /* Get whether the board has a distinct border or not. */

    if (!std::getline(config, line)) {
        std::println(stderr, "Unexpected configuration file end.");
        return 1;
    }

    bool borderedBoard { line == "y" };

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
    Camera cam { &robot };

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
