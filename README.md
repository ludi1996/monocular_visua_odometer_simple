This is an OpenCV 3.0 based implementation of a monocular visual odometry algorithm.

# monocular_visua_odometer_simple

very simple very naive monocular visua odometer.

Most code was copied from [Avi Singh's project](https://github.com/avisingh599/mono-vo).

I edited the detailed comments to give beginners a better understanding of the code.

# Requirement

OpenCV 3.0

# How to run

```bash
cd PATH/to/this/dir
mkdir build
cd build
cmake ..
make
./vo
```

# Database

I have provided some images from KITTI's Visual Odometry Dataset.  You can use your own data or get more from  [KITTI's Visual Odometry Dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php).

In order to run this algorithm on your own data, you must modify the intrinsic calibration parameters in the code.