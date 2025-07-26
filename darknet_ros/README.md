# darknet\_ros

Darknet YOLO real-time object detection contained in a ROS1 Wrapper.

## What it does?
- Detects objects in real-time using YOLO (v3/4/5/7)
- Takes advantage of CUDA for GPU acceleration
- Can be configured using ROS parameters and `.yaml` files

## Contents


## To install

### Prerequisites
- ROS installation (Melodic or Noetic)
- OpenCV
- CMake
- Existing catkin workspace (for demonstration purposes, `catkin_ws` is used)

### To build

```bash
cd ~/catkin_ws/src
git clone https://github.com/leggedrobotics/darknet_ros.git
rosdep install --from-paths . --ignore-src -r -y
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Using darknet_ros

### Getting Weights and Config

-Place YOLO config files, weights, and label files in `yolo_network_config/weights`

-Example

```bash
cd ~/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights
# Download YOLOv3 pre-trained weights
e.g. wget https://pjreddie.com/media/files/yolov3.weights

# Download YOLOv3-tiny .conv.15 pre-trained convolutional layers (for partial training)
wget https://raw.githubusercontent.com/hamzaMahdi/darknet/master/yolov3-tiny.conv.15
```

For `.cfg` and `.names` files, place them in `yolo_network_config`, or if you have custom paths, point to it using launch parameters.

### Run Detection

```bash
roslaunch darknet_ros darknet_ros.launch
```

Override parameters at launch:

```bash
roslaunch darknet_ros darknet_ros.launch image:=/camera/image_raw threshold:=0.6 gpu_id:=0
```

## Configuration

Parameters can be set via YAML or launch arguments:

| Parameter       | Type   | Default                 | Description                    |
| --------------- | ------ | ----------------------- | ------------------------------ |
| `model_cfg`     | string | `*.cfg`                 | Path to Darknet config file    |
| `model_weights` | string | `*.weights`             | Path to Darknet weights file   |
| `class_labels`  | string | `*.names`               | Path to class labels file      |
| `gpu_id`        | int    | `-1`                    | GPU device ID (`-1` for CPU)   |
| `threshold`     | double | `0.5`                   | Detection confidence threshold |
| `image_topic`   | string | `/camera/rgb/image_raw` | Input image topic              |

## Topics

| Topic                          | Type                             | Description                 |
| ------------------------------ | -------------------------------- | --------------------------- |
| `/darknet_ros/bounding_boxes`  | `darknet_ros_msgs/BoundingBoxes` | Detected boxes & classes    |
| `/darknet_ros/detection_image` | `sensor_msgs/Image`              | Annotated image with boxes  |
| `/darknet_ros/found_object`    | `std_msgs/Bool`                  | True if any object detected |
| `image_topic` (configurable)   | `sensor_msgs/Image`              | Raw image feed              |

## Launch Files

* **darknet\_ros.launch**: Main detection node
* **dabai_u3.launch**: Camera Module

## Terminal Quickstart

Follow these steps in separate terminals to build, run detection, capture images, and overlay boundaries:

1. **Clone & build** your workspace:

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/leggedrobotics/darknet_ros.git
   rosdep install --from-paths . --ignore-src -r -y
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

2. **Download YOLO weights**:

   ```bash
   cd ~/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights
   wget https://pjreddie.com/media/files/yolov3.weights
   wget https://raw.githubusercontent.com/hamzaMahdi/darknet/master/yolov3-tiny.conv.15
   ```

3. **Launch YOLO detection**:

   ```bash
   roslaunch darknet_ros darknet_ros.launch
   ```

4. **Capture camera frames** (optional):

   ```bash
   rosrun image_view image_saver \
     image:=/camera/rgb/image_raw \
     _filename_format:=frame_%04d.jpg \
     _sec_per_frame:=1
   ```

5. **Make boundary script executable**:

   ```bash
   chmod +x ~/catkin_ws/src/darknet_ros/scripts/boundary.py
   ```

6. **Run boundary overlay**:

   ```bash
   rosrun darknet_ros boundary.py \
     image_topic:=/camera/rgb/image_raw \
     boxes_topic:=/darknet_ros/bounding_boxes
   ```

## Examples

**On camera**:

```bash
roslaunch astra_camera dabai_u3.launch
```

**Run on rosbag**:

```bash
rosbag play sample.bag
roslaunch darknet_ros darknet_ros.launch
```

## Capturing Images & Running `boundary.py`

To save raw camera frames and overlay YOLO detections via `boundary.py`, follow these steps:

1. **Install the image view tool** (if not already):

   ```bash
   sudo apt-get update
   sudo apt-get install ros-melodic-image-view
   ```

2. **Capture images** from your camera topic:

   ```bash
   # Save one image per second with filenames frame_0001.jpg, frame_0002.jpg, etc.
   rosrun image_view image_saver image:=/camera/rgb/image_raw _filename_format:=frame_%04d.jpg _sec_per_frame:=1
   ```

   Let it run until you have the desired snapshots then Ctrl+C to stop.

3. **Add `boundary.py`** to your package's `scripts/` directory:

 
4. **Make it executable**:

   ```bash
   chmod +x scripts/boundary.py
   ```

5. **Run the boundary node**:

   In one terminal, launch YOLO detection:

   ```bash
   roslaunch darknet_ros darknet_ros.launch
   ```

   In another terminal, start the boundary overlay:

   ```bash
   rosrun darknet_ros boundary.py image_topic:=/camera/rgb/image_raw boxes_topic:=/darknet_ros/bounding_boxes
   ```

You should see an OpenCV window showing each frame with detected object boundaries overlaid.

## Troubleshooting

* **No detections**: Verify `image_topic` and active camera feed.
* **GPU not used**: Check CUDA installation and set `gpu_id` appropriately.
* **Build errors**: Ensure OpenCV and CUDA paths are correctly configured in `CMakeLists.txt`.

## Contributing

Contributions welcome! Please:

1. Fork the repository
2. Create a branch for your feature or bugfix
3. Implement changes with tests
4. Submit a pull request with description and issue reference

## License

MIT License. See the [LICENSE](LICENSE) file for details.
