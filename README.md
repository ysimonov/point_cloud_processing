# Point Cloud Processing

### Required, Recommended and Optional Libraries
- `sudo apt install libflann-dev`
- Recommended: `Lapack 3.10`
- Recommended: `OpenBLAS 0.3.21`
- Recommended: `Boost 1.80.0`
- Required: `Eigen 3.4.0`
- Required: `VTK 9.2.2`
- Required: `PCL 1.12.1` Note: cannot visualize with release version, instead build from master
- Currently Optional: `ros2 humble` (TODO)

### Build Instructions
- In the current directory `mkdir build && cd build`
- `cmake ..`
- `make`

### Run (Inside of /build Folder)
- `./object_detection`
- Press `Space` to move to the next frame, `Q` to quit

### Examples (Worst Cases)
![image1](https://github.com/ysimonov/point_cloud_processing/blob/main/images/1.png)
![image2](https://github.com/ysimonov/point_cloud_processing/blob/main/images/2.png)
![image3](https://github.com/ysimonov/point_cloud_processing/blob/main/images/3.png)
![image4](https://github.com/ysimonov/point_cloud_processing/blob/main/images/4.png)
![image5](https://github.com/ysimonov/point_cloud_processing/blob/main/images/recording.gif)
