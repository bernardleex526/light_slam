# M20Pro interface adaptation

The M20 configuration consumes `/LIDAR/POINTS` (`PointCloud2`) and `/IMU`
(`Imu`). Official RoboSense clouds use `x/y/z/intensity=float32`,
`ring=uint16`, and `timestamp=float64` absolute seconds. Lightning-lm's
RoboSense preprocessor converts `timestamp` to relative milliseconds internally.

Run `python3 launch/m20_interface_check.py` during `ros2 bag play` as a
read-only input gate before starting `m20_slam.launch.py`.
