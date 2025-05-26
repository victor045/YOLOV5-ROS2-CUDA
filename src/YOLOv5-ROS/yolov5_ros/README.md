
# 🧠 YOLOv5 ROS 2 Node for Real-Time Object Detection

This package integrates **YOLOv5** with **ROS 2** (Robot Operating System) for performing real-time object detection from a camera stream. It uses CUDA acceleration (if available) and publishes both bounding boxes and processed images through ROS topics.

---

## 📦 Overview

This node performs the following tasks:

1. Subscribes to a camera feed (`/image_raw`)
2. Uses a YOLOv5 model (e.g. `yolov5s.pt`) to detect objects
3. Publishes:
   - `/yolov5/bounding_boxes`: Detected objects as structured messages
   - `/yolov5/image_raw`: The original image (optional, or for chaining)

---

## 🧠 Architecture

- **YOLOv5 Inference**: Torch + DetectMultiBackend
- **ROS2 Integration**: Using `rclpy` and custom messages
- **Visualization**: Optionally opens a live `cv2.imshow()` window
- **Message types**:
  - Input: `sensor_msgs/Image`
  - Output: `bboxes_ex_msgs/BoundingBoxes`, `sensor_msgs/Image`

---

## 🚀 Topics

| Topic Name                 | Message Type                | Description                           |
|---------------------------|-----------------------------|---------------------------------------|
| `/image_raw`              | `sensor_msgs/Image`         | Input camera feed                     |
| `/yolov5/bounding_boxes`  | `bboxes_ex_msgs/BoundingBoxes` | Output detections (bounding boxes) |
| `/yolov5/image_raw`       | `sensor_msgs/Image`         | Re-published raw image (optional)     |

---

## ⚙️ Parameters

| Parameter       | Default                     | Description                          |
|----------------|-----------------------------|--------------------------------------|
| `weights`      | `config/yolov5s.pt`          | Path to YOLOv5 model weights         |
| `data`         | `data/coco128.yaml`          | Path to dataset YAML file            |
| `imagez_height`| `640`                        | Input image height                   |
| `imagez_width` | `640`                        | Input image width                    |
| `conf_thres`   | `0.25`                       | Confidence threshold                 |
| `iou_thres`    | `0.45`                       | IoU threshold for NMS                |
| `max_det`      | `1000`                       | Max number of detections             |
| `device`       | `'cpu'`                      | CUDA device (`'0'` for GPU)          |
| `view_img`     | `True`                       | Whether to open OpenCV image window  |
| `half`         | `False`                      | Use FP16 precision (if supported)    |

---

## 📂 Folder Structure

```bash
yolov5_ros/
├── yolov5_node.py        # ROS2 node using YOLOv5
├── utils/                # YOLOv5 utility functions
├── models/               # YOLOv5 backbone
├── data/                 # Data config files
├── config/               # Path to weight file (e.g., yolov5s.pt)
```

---

## 🧪 Example Output (Printed)

```
start ==================
['person', 'bottle'] [0.91, 0.72] [54, 222] [100, 230] [320, 280] [400, 315]
end ====================
```

---

## 🧰 Dependencies

- ROS 2 Foxy or newer
- Python 3.8+
- OpenCV
- Torch
- `cv_bridge`
- `bboxes_ex_msgs` custom message package

---

## 🧠 How it Works

1. Image is resized & normalized.
2. YOLOv5 detects bounding boxes.
3. Outputs are filtered via non-maximum suppression (NMS).
4. Results are scaled and converted to ROS messages.
5. Optionally shown with OpenCV (`cv2.imshow()`).

---

## 🏁 Launching

Make sure to:
- Build your ROS2 workspace
- Source the `install/setup.bash`
- Run:

```bash
ros2 run yolov5_ros yolov5_node
```

---

## 📄 License

MIT License

---

## 👤 Author

Víctor Fernando Pérez García  
GitHub: [victor045](https://github.com/victor045)
