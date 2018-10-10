# pointcloud_registration_icp
Point cloud registration using Iterative Closest Point (ICP) algorithm: c++ implementation


./icp_demo ../data/cars_1.pcd


## Dependencies
* Ubuntu16.04
* PCL1.7
* cmake

## Build

```bash
.../$ mkdir build && cd build
.../build$ cmake ..
.../build$ make
```

## Run
```bash
.../build$ ./icp_demo [data_file_path]
# ./icp_demo ../data/cars_1.pcd
```