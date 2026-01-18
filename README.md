# Chess Arm

### Build Instructions on the Raspberry Pi

```shell
# Requires cmake >= 4.1, tested on Raspberry Pi Zero 2W.

sudo apt install gstreamer1.0-libcamera gstreamer1.0-plugins-base \
  libopencv-dev libstdc++-15-dev clang-19 clang-tools-19 \
  --no-install-recommends

git clone https://github.com/a-manthan-t/chess_arm.git && cd chess_arm
mkdir build && cd build
cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=clang++-19 \
  -DCMAKE_CXX_COMPILER_CLANG_SCAN_DEPS=clang-scan-deps-19 ..
ninja
```

TODO:
- consider max and min angles for servos and torque limits
- spawn threads in main file and dispatch angles
- path, camera and arm tests
- joint loading from files
- Collision detection
- Camera integration (calibration, checkpoint generation, etc.)
