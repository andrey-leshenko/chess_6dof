# Realtime Chessboard Tracking

Get started by reading the [Full Guide](docs/guide.md).

This repository contains code for tracking a chessboard in realtime using two cameras and triangulation.
The position and rotation of the chessboard are calculated at each step. The rotation is found using the Kabsch SVD method.
The image processing is done using OpenCV, and the output is visualized using the cv::viz module.

* Under `camutils/` are programs for doing internal calibration and measuring camera latency and framerate.
* Under `calib_data/` are the internal calibration files for our Sony PlayStationEye cameras.
* Under `track/` if the main chessboard tracking program.
* Under `docs/` is our guide that explains the theory behind this project.

Written by Andrey Leshenko, Eli Tarnarutsky and Shir Amir.
