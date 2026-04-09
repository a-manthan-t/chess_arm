module;

#include <chrono>
#include <mutex>
#include <ranges>
#include <thread>

#include <opencv2/opencv.hpp>

// Handles taking and processing camera input.
module camera;

import arm;
import path;
import quaternion;

namespace camera {
    // Convert a piece type into a character to draw.
    std::string pieceToString(chess::Piece piece) {
        switch (piece.type()) {
            case 0: return piece.color() == chess::Color::WHITE ? "P" : "p"; // Pawn
            case 1: return piece.color() == chess::Color::WHITE ? "N" : "n"; // Knight
            case 2: return piece.color() == chess::Color::WHITE ? "B" : "b"; // Bishop
            case 3: return piece.color() == chess::Color::WHITE ? "R" : "r"; // Rook
            case 4: return piece.color() == chess::Color::WHITE ? "Q" : "q"; // Queen
            case 5: return piece.color() == chess::Color::WHITE ? "K" : "k"; // King
            default: return " "; // Empty
        }
    }

    // Detect the grid on a chess board set to the starting position.
    // Ensure MINIMAL SHADOWS and center the pieces in their squares!!
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
                    constexpr int SQ_POINT_SEPARATION { 25 * 25 };
                    float dx { p.x - x }, dy { p.y - y };

                    if (dx * dx + dy * dy <= SQ_POINT_SEPARATION) { // Each point should be at least 25 pixels away from any other.
                        duplicate = true;
                        break;
                    }
                }

                if (!duplicate) {
                    points.emplace_back(x, y);
                }
            }
        }

        if (points.size() < 81) {
            std::println(stderr, "Configuration failed because only {}/81 points detected.", points.size());
            return false;
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

        while (i < squares.size()) {
            cv::Point2f tl { points[j * colCount + k] }, br { points[(j + 1) * colCount + k + 1] }, vec { (br - tl) * 0.2 };
            squares[i++] = { tl + vec, br - vec };

            if (++k == 8 + firstCol) {
                k = firstCol;
                j--;
            }
        }

        if (std::pair bitboard { processRaw() }; bitboard == STARTING_BOARD) {
            std::string uci { chessEngine.getMove("") };
            board.makeMove(chess::pgn::uci::uciToMove(board, uci));
            whitesPerspective = true;
            generateCheckpoints(uci, chess::Move::NORMAL);
            return true;
        } else if (bitboard == FLIPPED_STARTING_BOARD) {
            std::ranges::reverse(squares);
            return true;
        }

        std::println(stderr, "Configuration failed because the board is not in the starting position.");
        return false;
    }

    // Take an image and detect the squares taken up by white
    // and black pieces, returning a bitboard representation.
    std::pair<uint64_t, uint64_t> Camera::processRaw() {
        if (!camera.read(raw)) {
            return {};
        }

        frame = raw.clone();

        cv::Mat hsv;
        cv::cvtColor(raw, hsv, cv::COLOR_BGR2HSV);
        cv::Scalar mean {}, std {};
        std::pair<uint64_t, uint64_t> bitboard {}; // first is for white pieces, second is for black

        for (int rank { 7 }; rank >= 0; --rank) {
            for (int file { 7 }; file >= 0 ; --file) {
                int square { rank * 8 + file }, colour {};
                constexpr double OCCUPIED_THRESHOLD { 25 }, WB_THRESHOLD { 85 };

                bitboard.first <<= 1;
                bitboard.second <<= 1;

                cv::meanStdDev(hsv(squares[square]), mean, std);

                if (std.val[2] > OCCUPIED_THRESHOLD) {
                    if ((mean.val[0] + mean.val[1] + mean.val[2]) / 3 > WB_THRESHOLD) {
                        bitboard.first++;
                        colour = 1;
                    } else {
                        bitboard.second++;
                        colour = -1;
                    }
                }

                rectangle(frame, squares[square], !colour ? BLUE : colour == 1 ? WHITE : BLACK, 2);
                putText(
                    frame,
                    pieceToString(board.at({ square })),
                    squares[square].tl() * 0.67 + squares[square].br() * 0.33,
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    GREEN
                );
            }
        }

        return bitboard;
    }

    std::string Camera::checkChange(const std::pair<uint64_t, uint64_t>& newBitboard) {
        if (board.us(chess::Color::WHITE).getBits() == newBitboard.first && board.us(chess::Color::BLACK).getBits() == newBitboard.second) {
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

            if (board.us(chess::Color::WHITE).getBits() == newBitboard.first && board.us(chess::Color::BLACK).getBits() == newBitboard.second) {
                return chess::pgn::uci::moveToUci(move); // Return the move that was made.
            }

            board.unmakeMove(move);
        }

        return "I"; // Invalid change
    }

    // Converts a given chess move into targets to move
    // the robot arm to, and enqueues these targets.
    void Camera::generateCheckpoints(const std::string &move, MoveType moveType) const {
        std::array<int, 2>
            sourceSquare { move[0] - 'a', move[1] - '1' },
            targetSquare { move[2] - 'a', move[3] - '1' };

        if (!whitesPerspective) {
            sourceSquare = { 7 - sourceSquare[0], 7 - sourceSquare[1] };
            targetSquare = { 7 - targetSquare[0], 7 - targetSquare[1] };
        }

        path::Checkpoint
            start { robot->getLatestCheckpoint() },
            above {
                {
                    quaternion::vector((targetSquare[0] + 0.5) * squareWidth, (targetSquare[1] + 0.5) * squareWidth, 2 * squareWidth),
                    quaternion::rotation(0, 1, 1) // TODO
                }, 2
            },
            grabLevel {
                {
                    above.orientation.position - quaternion::vector(0, 0, -1.5 * squareWidth),
                    quaternion::rotation(0, 1, 1) // TODO
                }, 0
            },
            dump {
                {
                    quaternion::vector(-squareWidth, squareWidth * 4, 0.5),
                    quaternion::rotation(0, 1, 1) // TODO
                }, 0
            };

        switch (moveType) {
            case chess::Move::PROMOTION:  // TODO alert replacement with queen?
            case chess::Move::NORMAL: {
                if (board.occ().check(targetSquare[0] + targetSquare[1] * 8)) {
                    robot->addCheckpoint(above);
                    robot->addCheckpoint(grabLevel);
                    robot->addCheckpoint(grabLevel); // same checkpoint twice means toggle grip
                    robot->addCheckpoint(above);
                    robot->addCheckpoint(dump);
                    robot->addCheckpoint(dump);
                }
                break;
            }
            case chess::Move::ENPASSANT: {
                path::Checkpoint
                    pawnAbove {
                        {
                            quaternion::vector((targetSquare[0] + 0.5) * squareWidth, (targetSquare[1] - 0.5) * squareWidth, 2 * squareWidth),
                            quaternion::rotation(0, 1, 1) // TODO
                        }, 2
                    },
                    pawnGrab {
                        {
                            pawnAbove.orientation.position - quaternion::vector(0, 0, -1.5 * squareWidth),
                            quaternion::rotation(0, 1, 1) // TODO
                        }, 0
                    };

                robot->addCheckpoint(pawnAbove);
                robot->addCheckpoint(pawnGrab);
                robot->addCheckpoint(pawnGrab);
                robot->addCheckpoint(pawnAbove);
                robot->addCheckpoint(dump);
                robot->addCheckpoint(dump);
                break;
            }
            case chess::Move::CASTLING: {
                int
                    rookX { targetSquare[0] < 4 ? 0 : 7 },
                    rookTX { targetSquare[0] < 4 ? targetSquare[0] + 1 : targetSquare[0] - 1 };

                path::Checkpoint
                    rookAbove {
                        {
                            quaternion::vector((rookX + 0.5) * squareWidth, (targetSquare[1] - 0.5) * squareWidth, 2 * squareWidth),
                            quaternion::rotation(0, 1, 1) // TODO
                        }, 2
                    },
                    rookGrab {
                        {
                            rookAbove.orientation.position - quaternion::vector(0, 0, -1.5 * squareWidth),
                            quaternion::rotation(0, 1, 1) // TODO
                        }, 0
                    },
                    rookDropAbove {
                        {
                            quaternion::vector((rookTX + 0.5) * squareWidth, (targetSquare[1] - 0.5) * squareWidth, 2 * squareWidth),
                            quaternion::rotation(0, 1, 1) // TODO
                        }, 2
                    },
                    rookDropGrab {
                        {
                            rookDropAbove.orientation.position - quaternion::vector(0, 0, -1.5 * squareWidth),
                            quaternion::rotation(0, 1, 1) // TODO
                        }, 0
                    };

                robot->addCheckpoint(rookAbove);
                robot->addCheckpoint(rookGrab);
                robot->addCheckpoint(rookGrab);
                robot->addCheckpoint(rookAbove);
                robot->addCheckpoint(rookDropAbove);
                robot->addCheckpoint(rookDropGrab);
                robot->addCheckpoint(rookDropGrab);
                robot->addCheckpoint(rookDropAbove);
                break;
            }
            default: std::println(stderr, "Invalid move type received: {}", move); // Should not happen.
        }

        robot->addCheckpoint(above);
        robot->addCheckpoint(grabLevel);
        robot->addCheckpoint(grabLevel);
        robot->addCheckpoint(above);
        robot->addCheckpoint(start);
        robot->addCheckpoint(start);
    }

    // For streaming purposes.
    void Camera::encodeFrame() {
        if (!frame.empty()) {
            std::lock_guard lock { cameraMutex }; // Lock so streamer doesn't read a partially ready image.
            cv::imencode(".jpg", frame, buffer);
        }
    }

    // The main loop of the camera which processes captured
    // images, generates moves, and commands the arm.
    [[noreturn]] void Camera::loop() {
        do {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        } while (!configure());

        while (true) {
            if (!robot->moving) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            switch (std::string move { checkChange(processRaw()) }; move[0]) {
                case 'I': // TODO communicate error to server...
                    std::println(stderr, "Invalid board position detected.");
                    break;
                case 'N':
                    break;
                default:
                    if (board.isGameOver().second == chess::GameResult::NONE) {
                        std::string uci { chessEngine.getMove(move) };
                        chess::Move engineMove { chess::pgn::uci::uciToMove(board, uci) };
                        board.makeMove(engineMove);
                        generateCheckpoints(uci, engineMove.typeOf());
                    }

                    break;
            }

            encodeFrame();

            if (board.isGameOver().second != chess::GameResult::NONE) {
                break; // TODO notify end and colour won/draw?
            }
        }
    }
}
