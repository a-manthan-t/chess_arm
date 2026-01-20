module;

#include <opencv2/opencv.hpp>

export module camera;

export namespace camera {
    class Camera {
        static bool initialised;
        static float time;
        static cv::VideoCapture camera; // We only want one camera.
        static cv::Mat frame;

        Camera();

        public:
            void initialise(unsigned int maxFps);
    };
}
