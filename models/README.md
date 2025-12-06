# Models – YOLOv8s and Hailo `.hef` Files

This project uses a **custom YOLOv8s model** running on a Hailo-8L
accelerator to detect a white cylindrical target on the ground (which
appears as a white circular region from above).

## `.hef` Files

The compiled Hailo model (`.hef`) is **not** included in this repository.

Reasons:

- Model binaries are hardware- and toolchain-specific.
- They may be large and not suitable for version control.
- Distribution can be restricted by licensing or competition rules.

Instead, this repository expects that you provide your own `.hef` file
compatible with your Hailo-8L runtime.

## Expected Usage

In the original setup, a custom YOLOv8s model for the target was compiled
to a Hailo `.hef` file (for example, `yolov8s_circle.hef`) and placed in a
known location on the Raspberry Pi 5. The vision pipeline then loaded this
file via the Hailo/GStreamer pipeline.

To adapt this project to your environment:

1. Train or obtain a YOLOv8s (or similar) model for your target object.
2. Use the Hailo toolchain (Model Zoo / TAPPAS / compiler) to generate a
   `.hef` file for your Hailo-8L device.
3. Place the `.hef` file in a suitable location on the Raspberry Pi 5.
4. Update the relevant configuration or code (e.g. in `vision_node.py` or
   a parameter file) so that it points to your `.hef` file.

## Scope

This repository focuses on the **autonomy logic and integration**:
- ROS 2 nodes
- PX4/MAVROS interaction
- HIL testing structure

It does **not** cover:
- Full training pipelines
- Dataset preparation
- Hailo model compilation tutorials

For model training and compilation details, refer to the Hailo documentation
and example projects.
