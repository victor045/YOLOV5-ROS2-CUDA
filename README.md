
# 🧠 YOLOv5 ROS 2 Inference System with CUDA

This repository contains a complete ROS 2 (Foxy/Humble) integration of YOLOv5 for real-time object detection using camera input and GPU-accelerated inference. The node supports custom model weights, publishes bounding boxes, and visualizes results in real-time.

---

## 📦 Project Features

- ✅ Real-time detection using YOLOv5 (Nano or custom weights)
- ⚡ GPU acceleration (CUDA + FP16 support)
- 🧠 Uses Ultralytics' `DetectMultiBackend` for flexibility
- 🎯 Outputs: bounding boxes + confidence
- 🔁 Subscribes to: `/image_raw`
- 📤 Publishes:
  - `/yolov5/image_raw`: annotated camera feed
  - `/yolov5/bounding_boxes`: detection results as custom messages
- 🧪 Modular + benchmark-friendly

---

## 🗂 Directory Structure

```
yolov5_ros/
├── main.py              # Full ROS2 node with DetectMultiBackend integration
├── yolov5_node.py       # Simpler YOLOv5 inference node
├── __init__.py
├── config/              # Place to store model weights (e.g. yolov5s.pt)
├── launch/              # Optional: launch files
├── models/, utils/      # YOLOv5 model code (optional submodules)
```

---

## 🚀 Installation

### 1. Clone the Repo

```bash
mkdir ws_yolov5
cd ~/ws_yolov5
git clone https://github.com/victor045/YOLOV5-ROS2-CUDA
```

### 2. Build the Workspace

```bash
cd ~/ws_yolov5
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🧠 Running the Main Node

### Option A: Using main file for deployment

To launch a yolov5 model suited for most dev boards in deployment mode.
Run:

```bash
source install/setup.bash
ros2 run yolov5_ros yolov5_ros 
```
or 

```bash
source install/setup.bash
ros2 launch yolov5_ros yolov5s_simple.launch.py 
```


### Option B: For prototyping and better performance

You can execute a single yolov5 node with video streaming for prototyping.
Run:

```bash
source install/setup.bash
ros2 run yolov5_ros yolov5_node
```

> Make sure to source ROS 2 and your workspace before running.

---

## 🧪 Inputs and Outputs

### 🔄 Topics

| Topic                      | Type                    | Description                      |
|---------------------------|-------------------------|----------------------------------|
| `/image_raw`              | `sensor_msgs/Image`     | Input video feed (e.g. from camera) |
| `/yolov5/image_raw`       | `sensor_msgs/Image`     | Output: annotated image          |
| `/yolov5/bounding_boxes`  | `bboxes_ex_msgs/BoundingBoxes` | Output: detection data          |

---

## 🧰 Parameters (main.py)

Declared via ROS 2 params or `launch.py`:

| Name             | Type    | Default             | Description                     |
|------------------|---------|---------------------|---------------------------------|
| `weights`        | string  | yolov5s.pt          | Path to YOLOv5 weights          |
| `conf_thres`     | float   | 0.25                | Confidence threshold            |
| `iou_thres`      | float   | 0.45                | IoU threshold for NMS           |
| `imagez_height`  | int     | 640                 | Image height                    |
| `imagez_width`   | int     | 640                 | Image width                     |
| `device`         | string  | `'cuda'` or `'cpu'` | Inference device                |

---

## 🖥 Visualization

```bash
ros2 run rqt_image_view rqt_image_view
```

Select topic:
```
/yolo/image_result
```

---

## 🧪 Benchmarking

Modify `main.py` or `yolov5_node.py` to include timing:

```python
start = time.time()
results = model(image)
print("Inference time:", (time.time() - start) * 1000, "ms")
```

---

## 🛠 Known Issues

- ⚠️ You must `source install/setup.bash` or `ros2 run` will not find the node
- ⚠️ Custom messages (`bboxes_ex_msgs`) must be built in the same workspace
- ⚠️ For direct Python execution, make sure to fix PYTHONPATH or use `-m`

---

## 📜 License

MIT License © 2024

---

## 🤝 Contributing

Pull requests and bug reports welcome. Please create an issue first before proposing large changes.

---

## 📧 Contact

Maintainer: Victor Perez
GitHub: [https://github.com/victor045](https://github.com/victor045)

