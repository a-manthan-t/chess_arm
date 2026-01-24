# Chess Arm

### Running the Streaming Server

// Info here

### Build Instructions on the Raspberry Pi

```shell
# Requires cmake >= 4.1, tested on the Raspberry Pi Zero 2W
# running the Raspberry Pi OS Lite.

sudo apt install gstreamer1.0-libcamera gstreamer1.0-plugins-base \
  libopencv-dev libstdc++-15-dev clang-19 clang-tools-19 \
  --no-install-recommends

git clone https://github.com/a-manthan-t/chess_arm.git && cd chess_arm
mkdir build && cd build

# Add the flag -DTESTING=1 for a test build instead of production, and
# -DPC_BUILD=1 if not building on a Raspberry Pi to autodetect the camera.
cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=clang++-19 \
  -DCMAKE_CXX_COMPILER_CLANG_SCAN_DEPS=clang-scan-deps-19 ..
ninja
```

### Setting Up `config.txt`

// Info here

### Third-Party Libraries

##### C++

- [OpenCV](https://opencv.org)
- [Doctest](https://github.com/doctest/doctest)
- [easywsclient](https://github.com/dhbaird/easywsclient)

##### JavaScript

- [express](https://www.npmjs.com/package/express)
- [uuid](https://www.npmjs.com/package/uuid)
- [ws](https://www.npmjs.com/package/ws)


- test joint loading from files

TODO:
- consider max and min angles for servos and torque limits
- spawn threads in main file and dispatch angles
- path, camera, streamer and arm, main tests
- Collision detection
- Camera integration (calibration, checkpoint generation, etc.)
- Documentation (overview of system architecture, etc.)
- Webserver test js
- Stop checkpoint generation from quat
