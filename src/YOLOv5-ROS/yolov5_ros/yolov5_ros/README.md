
# 🚀 YOLOv5 ROS 2 Lightweight Node

This ROS 2 node performs real-time object detection using the **YOLOv5n** model (nano version), with a simple architecture based on `torch.hub` and minimal dependencies. It is intended for lightweight or quick-deploy scenarios.

---

## 🧠 What This Code Does

- Subscribes to a ROS 2 image stream (`/image_raw`)
- Performs object detection using YOLOv5n from `ultralytics/yolov5`
- Draws bounding boxes on the image
- Publishes:
  - `/bounding_boxes` as `BoundingBoxes`
  - `/yolo/image_result` with drawn image

---

## 📂 ROS Topics

| Topic                  | Type                          | Description                      |
|------------------------|-------------------------------|----------------------------------|
| `/image_raw`           | `sensor_msgs/msg/Image`       | Input video frame from camera    |
| `/bounding_boxes`      | `bboxes_ex_msgs/BoundingBoxes`| Detected object info             |
| `/yolo/image_result`   | `sensor_msgs/msg/Image`       | Image with annotations           |

---

## 🔍 Key Features

- 🔌 **Plug-and-play**: Uses `torch.hub.load` to automatically download YOLOv5n
- ⚡ **CUDA Support**: Automatically uses GPU if available
- 📷 **Minimal Setup**: No need for custom dataset, just launch and stream
- 🔁 **Live Preview**: Displays results using OpenCV

---

## ✅ Differences vs `main.py`

| Feature                     | `yolov5_node.py`                            | `main.py`                                                                 |
|----------------------------|---------------------------------------------|---------------------------------------------------------------------------|
| **Model loading**          | Uses `torch.hub.load()`                     | Uses `DetectMultiBackend` for flexible backends (ONNX, TensorRT, etc.)   |
| **Model size**             | YOLOv5n (nano, pretrained from Ultralytics) | Any YOLOv5 model (`.pt`) you specify                                     |
| **Flexibility**            | Fixed configuration                         | Fully parameterized via ROS params                                       |
| **ROS parameters**         | None                                        | Yes (e.g., `conf_thres`, `device`, etc.)                                 |
| **Visualization**          | Shows `cv2.imshow()` per frame              | Optional based on `view_img` param                                       |
| **Code length**            | Simple, ~100 lines                          | Complex, modular with `yolov5_demo` class                                |
| **Dependencies**           | Just `torch`, `cv2`, ROS 2, `cv_bridge`     | Requires full YOLOv5 repo integrated                                     |

---

## 📦 Dependencies

- ROS 2 Foxy or newer
- Python 3.8+
- OpenCV
- Torch
- cv_bridge
- bboxes_ex_msgs

---

## 🏁 How to Run

```bash
ros2 run yolov5_ros yolov5_node
```

> Replace `yolov5_ros` with the actual package name if different.

---

## 🧪 Example Output

```
YOLOv5 Detection
[class_id: 0] [xmin: 42, ymin: 63, xmax: 120, ymax: 190]
[class_id: 1] [xmin: 250, ymin: 100, xmax: 300, ymax: 180]
```

---

## 📄 License

MIT License

---

## 👤 Author

Víctor Fernando Pérez García  
GitHub: [victor045](https://github.com/victor045)

