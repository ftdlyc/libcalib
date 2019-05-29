Libcalib
---
This repository contains some sources to calibrate stereo cameras.

#### Dependencies
- OpenCV
- Eigen3
- [Ceres](https://github.com/ceres-solver/ceres-solver)
- [Rapidjson](https://github.com/Tencent/rapidjson)
- [Libcbdetect](https://github.com/ftdlyc/libcbdetect)

#### Compilation
1. build libcbdetect
2. copy libcbdetect header files into libcalib/include/libcbdetect
3. copy rapidjson header files into libcalib/include/rapidjson
3. copy libcbdetect.so into libcalib/lib
4. build libcalib