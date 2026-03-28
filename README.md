[English] | [简体中文](README_CN.md)

# fisheye2panorama

This project converts fisheye camera images into 360° cylindrical panoramas using camera calibration and geometric projection. It can unwrap heavily distorted fisheye frames into more readable panoramic views in real time, suitable for robotics, surveillance, and computer vision tasks.

---

## Features

- **Real-time processing**: works with live streams (about 1 ms in the author's test environment).
- **ROS + OpenCV integration**: easy to deploy in robotic systems.
- **Configurable parameters**: customizable panorama resolution and mapping parameters.
- **Lightweight and efficient**: suitable for embedded or resource-limited platforms.

---

## Example

**Input fisheye image:**  
![Source Image](image/fisheye.png)

**Output 360° cylindrical panorama:**  
![Panorama Image](image/panorama.png)

**Video demo:**  
![Video Demo](image/demo.gif)

---

## Typical Applications

- **Autonomous navigation**: provides omnidirectional environmental perception for robots, drones, and autonomous vehicles.
- **Surveillance**: one fisheye camera can cover a wide area and reduce blind spots.
- **Obstacle detection and mapping**: can be integrated with vision algorithms for real-time perception.
- **Omnidirectional vision**: supports SLAM, visual odometry, and multi-camera fusion.
- **VR / immersive video**: converts fisheye footage into usable panoramic content.

---

## Installation and Run

Clone this repository into the `src` folder of your ROS workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/canyueduxuan/fisheye2panorama.git
cd ..
catkin_make
rosrun fisheye2panorama fisheye2panorama_node
```

## Testing

```bash
cd ~/catkin_ws/src/fisheye2panorama/dataset
rosbag play data.bag
```

## Real-World Deployment

Use [kalibr](https://github.com/ethz-asl/kalibr) to calibrate your fisheye camera with `eucm-none` parameters, then write them into `config/config.yaml`. Also ensure your ROS topic names and message types match your actual device setup.

Since earlier experiment records were lost, here is a simple demo of four virtual pinhole cameras after undistortion (similar to [vins-fisheye](https://github.com/xuhao1/VINS-Fisheye)):

![4 pinhole cams](image/demo2.gif)

You can see that with `eucm-none` (suitable for >180°), boundary ghosting artifacts are removed. In practice, `omni-radtan` (<180°) may still produce edge ghosting.

If calibration conditions are limited or camera quality is poor, increasing `cylinder_cy` in `config.yaml` to shift the cylindrical camera optical axis upward can improve results without reducing vertical FOV.

## Acknowledgement

Inspired by [vins-fisheye](https://github.com/xuhao1/VINS-Fisheye).

[3D Object Detection from a Single Fisheye Image Without a Single Fisheye Training Image](https://arxiv.org/abs/2003.03759) shows that cylindrical images have radial translation invariance (fisheye images do not), making them more suitable for CNN-based 3D object detection.

## License

[MIT LICENSE](LICENSE)

