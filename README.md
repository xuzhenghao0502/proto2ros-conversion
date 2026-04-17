# Protobuf与 ROS 2 互操作

![Python Support](https://img.shields.io/badge/python-3.8%20%7C%203.9%20%7C%203.10%20%7C%203.11%20%7C%203.12-blue)
![ROS Support](https://img.shields.io/badge/ROS-humble%20%7C%20jazzy-blue)

## 概述

`proto2ros` 在依赖 Protobuf 的代码与使用 ROS 2 的代码之间提供一层互操作能力：根据 Protobuf 消息定义生成对等的 ROS 2 消息定义，并在 C++、Python 等语言中生成双向转换 API。目前支持 Protobuf 语法 v2 与 v3，其中 **v3 经过更充分测试**。

## 软件包

本仓库包含以下 ROS 2 软件包：

| 软件包 | 说明 |
|--------|------|
| [`proto2ros`](proto2ros) | ROS 2 等价消息生成与转换代码生成的核心机制。 |
| [`proto2ros_tests`](proto2ros_tests) | 针对 `proto2ros` 生成消息与转换 API 的端到端测试。 |
| [`gpal_proto_bridge`](gpal_proto_bridge) | 基于 GPAL Protobuf 的 ROS 2 消息，以及 EkaRT SQLite 录制转 rosbag2 的工具。 |

## EkaRT SQLite 工具（`gpal_proto_bridge`）

执行 `colcon build` 并 `source install/setup.bash` 后，可通过 `ros2 run gpal_proto_bridge <脚本名>` 调用；也可在源码树中直接运行 `gpal_proto_bridge/scripts/` 下的脚本。

**常见依赖：** `python3-protobuf`、`python3-numpy`、`python3-opencv`、`ffmpeg`。若数据库中的相机流为 **H.264/H.265**，建议再执行 `pip install av`（PyAV），以便 **P/B 帧** 在预览或导出时能正确解码。

### `db_peek_camera_frame`

查看单条 `eka::rt::msg::CameraFrame` 记录，或 **列出全部录制通道**（原始 URL 与 `db2bag` 使用的 ROS 风格 topic 名）。

| 用途 | 命令 |
|------|------|
| 列出所有通道 | `ros2 run gpal_proto_bridge db_peek_camera_frame path/to/recording.db --list` |
| 将第 10 帧存为 PNG | `ros2 run gpal_proto_bridge db_peek_camera_frame path/to/recording.db --topic /ekart/shm/hal/compressed/cam_fcf_depth_5 --frame 10 -o frame10.png` |
| 在窗口中显示（需要 `DISPLAY`） | 同上，省略 `-o` 即可。 |

使用 `--list` 可复制准确的 `--topic` 字符串。H.264/H.265 解码会按顺序从第 1 包喂到第 *N* 包（**有状态解码**），与 `db2bag` 行为一致。

### `db2bag`

将整份 EkaRT `.db` 转为 rosbag2 目录（默认存储后端 `sqlite3`）。`CameraFrame` 行会变为 `sensor_msgs/msg/Image`，`encoding` 为 **`bgr8`**（按 OpenCV 常用的 **BGR** 布局解码）；时间戳使用 **`EkaRT_Header.start_timestamp`** 与每行的 **`elapsed`**。

| 用途 | 命令 |
|------|------|
| 整包转换 | `ros2 run gpal_proto_bridge db2bag path/to/recording.db path/to/out_bag_dir` |
| 使用 MCAP | 若输出需使用 MCAP，添加 `--storage mcap`。 |
| 仅导出指定 topic | `ros2 run gpal_proto_bridge db2bag recording.db out_bag_dir --only-topic /ekart/shm/hal/compressed/fisheye_fcf_depth_5`（可多次写 `--only-topic`）。topic 名须与 `db_peek_camera_frame --list` 中的规范化名称一致。若在 `db2bag` 上使用了 `--topic-prefix`，则每个 `--only-topic` 也要带上该前缀。 |

**示例（在仓库根目录、已 source 工作空间后）：**

```bash
ros2 run gpal_proto_bridge db_peek_camera_frame data/C2_0328.db --list
ros2 run gpal_proto_bridge db_peek_camera_frame data/C2_0328.db \
  --topic /ekart/shm/hal/compressed/cam_fcf_depth_5 --frame 10 -o data/frame10.png
ros2 run gpal_proto_bridge db2bag data/C2_0328.db data/export_bag \
  --only-topic /ekart/shm/hal/compressed/cam_fcf_depth_5
```

## 构建与环境依赖（Python）

`colcon build` 时 **`proto2ros` 会在当前环境的 Python 里执行生成脚本**，需安装下列模块（与 `proto2ros/package.xml` 中 `python3-*` 声明对应）。**`gpal_proto_bridge`** 另需 OpenCV 等与 ROS 一同提供的包。

| 模块 | Ubuntu/Debian 包名 | pip 包名 | 说明 |
|------|-------------------|----------|------|
| Jinja2 | `python3-jinja2` | `jinja2` | 生成 C++/Python 模板代码 |
| inflection | `python3-inflection` | `inflection` | 命名风格转换 |
| multipledispatch | `python3-multipledispatch` | `multipledispatch` | 转换分派 |
| NetworkX | `python3-networkx` | `networkx` | 依赖图与环检测 |
| NumPy | `python3-numpy` | `numpy` | `proto2ros` 兼容层 |
| Protobuf | `python3-protobuf` | `protobuf` | `google.protobuf` |
| PyYAML | `python3-yaml` | `PyYAML` | 读取 overlay 配置 |
| SciPy | `python3-scipy` | `scipy` | **未写在 package.xml**；在多数系统上 **`import networkx` 会间接加载 SciPy**，建议与 NumPy 一并用 **apt 或 pip 成套安装**，避免 ABI 混用（见下节）。 |
| OpenCV | `python3-opencv` | `opencv-python` | 主要用于 **`gpal_proto_bridge` 脚本**（`db2bag` / `db_peek_camera_frame`） |
| PyAV | `python3-av`（若有） | `av` | **可选**；H.264/H.265 有状态解码时建议 `pip install av` |

**推荐一次装齐（Ubuntu，与系统 Python 一致）：**

```bash
sudo apt install \
  python3-jinja2 python3-inflection python3-multipledispatch \
  python3-networkx python3-numpy python3-scipy python3-protobuf python3-yaml \
  python3-opencv ffmpeg
```

**或用 pip（须与执行 `colcon` 的 `python3` 为同一解释器）：**

```bash
python3 -m pip install jinja2 inflection multipledispatch networkx numpy scipy protobuf PyYAML
```

更省事的做法是在 workspace 根目录对源码跑一次 **`rosdep install --from-paths <你的src路径> --ignore-src -r -y`**（需已配置 ROS 与 rosdep），按各包 `package.xml` 拉取依赖；若仍缺 **`jinja2`** / **`scipy`** 等，再补装上面列表中的项。

## 构建故障排除

**`ValueError: numpy.dtype size changed ...`（出现在 `import networkx` / `scipy` 时）**  
说明当前 Python 里 **NumPy 与 SciPy 的 wheel/系统包混用**，ABI 不一致。`proto2ros` 会间接加载 `networkx` → `scipy`。

任选其一修复后重新 `colcon build`：

- 用 **pip成套升级**（与执行 `colcon` 的同一个 `python3`）：  
  `python3 -m pip install -U "numpy>=1.26" scipy networkx`
- 或 **只用 apt**，并避免再用 pip 覆盖系统 `numpy`：  
  `sudo apt install --reinstall python3-numpy python3-scipy python3-networkx`

**`ModuleNotFoundError: No module named 'jinja2' / 'inflection' / …`**  
说明 **「构建与环境依赖（Python）」** 一节中的包尚未装入当前 `python3`。优先执行该节的 **`apt install`** 或 **`pip install`** 整表命令，然后重新 `colcon build`。

## 后续步骤

请参阅 [贡献指南](CONTRIBUTING.md)。
