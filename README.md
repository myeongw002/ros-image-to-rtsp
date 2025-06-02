# ROS Image to RTSP Stream (C++)

This ROS package provides a node that subscribes to a camera image topic (`sensor_msgs/Image` or `sensor_msgs/CompressedImage`), decodes it with OpenCV, and streams the result to an RTSP server using FFmpeg.

## Features

- Supports both raw and compressed ROS image topics
- Fully configurable via a YAML config file (image size, topic name/type, RTSP server URL, etc.)
- Simple and efficient C++ implementation
- Compatible with VLC, ffplay, and any standard RTSP client

## Requirements

- ROS (tested on Melodic/Noetic)
- OpenCV (installed via ROS or system package)
- yaml-cpp (`sudo apt-get install libyaml-cpp-dev`)
- FFmpeg (`sudo apt-get install ffmpeg`)
- [rtsp-simple-server](https://github.com/aler9/rtsp-simple-server) or any RTSP server

## Configuration

Edit or create a file named `config.yaml`:

```yaml
rtsp_url: "rtsp://localhost:8554/stream"
topic: "/camera1/image_color/compressed"
topic_type: "compressed"   # "compressed" or "image"
width: 1024
height: 768
fps: 30
```

## Usage

1. **Build the package**:
    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

2. **Start your RTSP server** (example: rtsp-simple-server):
    ```bash
    ./rtsp-simple-server
    ```

3. **Run the node**:
    ```bash
    rosrun your_package_name compressedimage_to_rtsp_cpp _config:=/path/to/config.yaml
    ```

4. **View the stream** in VLC or ffplay:
    ```
    rtsp://localhost:8554/stream
    ```

## Notes

- Make sure the image size and topic type in `config.yaml` match your ROS camera topic.
- The code will automatically select the correct callback for raw or compressed image messages.
- The RTSP server must be running before starting the node.

## License

MIT License
