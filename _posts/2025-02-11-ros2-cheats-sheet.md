---
title: ROS2 Cheats Sheet (Humble)
description: >-
  ros2的命令参数使用示例.
author: xie
date: 2025-02-11 15:00:00 +0800
categories: [Basic, ROS2]
tags: [ROS2]
pin: true
media_subpath: '/posts/20250211'
---

# ROS2 CLI Cheatsheet (Humble)

ROS2 CLI 工具以 `ros2` 作为前缀，后跟命令、动词和（可能的）位置/可选参数。以下是 ROS 2 中一些常见的 CLI 工具及其详细用法。

## 获取帮助信息

### 获取命令行工具的帮助：

```bash
ros2 <command> --help
```

### 获取某个动词的帮助：

```bash
ros2 <command> <verb> -h
```

### 自动补全：

使用 `[tab][tab]` 可进行自动补全。例如：

```bash
ros2 <command> [tab][tab]
```

---

## `ros2 action`

ROS 2 的 action 是一种用于处理长期运行任务的通信机制。`ros2 action` 提供了与 action 相关的调试功能。

### 动词说明：

- `info`：显示某个 action 的信息。
- `list`：列出所有正在运行的 actions。
- `send_goal`：发送 action 目标。

### 示例：

- 获取 `/fibonacci` action 的信息：

  ```bash
  ros2 action info /fibonacci
  ```

- 列出所有可用的 actions：

  ```bash
  ros2 action list
  ```

- 发送目标给 `/fibonacci` action（例如，计算 Fibonacci 序列的前 5 个数字）：

  ```bash
  ros2 action send_goal /fibonacci action_tutorials/action/Fibonacci "{order: 5}"
  ```

---

## `ros2 bag`

`ros2 bag` 用于记录和回放话题数据。它是进行数据记录和调试的有力工具。

### 动词说明：

- `convert`：将 rosbag 文件从一个格式转换为另一个格式。
- `info`：查看 rosbag 文件的元数据信息。
- `list`：列出可用的插件。
- `play`：回放 rosbag 中记录的话题。
- `record`：开始记录话题数据。
- `reindex`：重建 rosbag 的索引文件。

### 示例：

- 查看 rosbag 文件的元数据：

  ```bash
  ros2 bag info <bag-name>
  ```

- 回放 rosbag 文件：

  ```bash
  ros2 bag play <bag-name>
  ```

- 记录所有话题数据到 rosbag 文件：

  ```bash
  ros2 bag record -a
  ```

- 重建 rosbag 索引：

  ```bash
  ros2 bag reindex <bag-dir>
  ```

---

## `ros2 component`

`ros2 component` 用于管理和操作 ROS 2 的组件化架构。组件是 ROS 2 中可动态加载和卸载的模块。

### 动词说明：

- `list`：列出当前运行的组件。
- `load`：将组件加载到容器节点中。
- `standalone`：将组件作为独立容器节点运行。
- `types`：列出所有已注册的组件类型。
- `unload`：从容器节点卸载指定组件。

### 示例：

- 列出当前所有组件：

  ```bash
  ros2 component list
  ```

- 加载名为 `composition::Talker` 的组件：

  ```bash
  ros2 component load /ComponentManager composition composition::Talker
  ```

- 卸载组件：

  ```bash
  ros2 component unload /ComponentManager 1
  ```

---

## `ros2 doctor`

`ros2 doctor` 用于检查 ROS 2 环境的健康状况，帮助诊断配置或运行问题。

### 参数说明：

- `--report` (`-r`)：输出所有检查项的结果。
- `--report-fail` (`-rf`)：仅输出检查失败的项。
- `--include-warning` (`-iw`)：将警告视为失败项。

### 示例：

- 运行诊断检查：

  ```bash
  ros2 doctor
  ```

- 输出详细报告：

  ```bash
  ros2 doctor --report
  ```

- 仅输出失败项：

  ```bash
  ros2 doctor --report-fail
  ```

- 包括警告项并输出失败报告：

  ```bash
  ros2 doctor --include-warning --report-fail
  ```

- 等价于运行 `ros2 doctor`：

  ```bash
  ros2 wtf
  ```

---

## `ros2 interface`

`ros2 interface` 用于查看和操作 ROS 2 的接口，包括话题、服务和动作。

### 动词说明：

- `list`：列出所有可用接口类型。
- `package`：列出某个包中提供的接口。
- `packages`：列出提供接口的所有包。
- `proto`：显示接口的协议文件（例如，消息、服务的定义）。
- `show`：显示接口的定义信息。

### 示例：

- 列出所有可用的接口：

  ```bash
  ros2 interface list
  ```

- 列出 `std_msgs` 包中提供的接口：

  ```bash
  ros2 interface package std_msgs
  ```

- 显示 `AddTwoInts` 服务的协议文件：

  ```bash
  ros2 interface proto example_interfaces/srv/AddTwoInts
  ```

- 显示 `geometry_msgs/msg/Pose` 消息类型的定义：

  ```bash
  ros2 interface show geometry_msgs/msg/Pose
  ```

---

## `ros2 launch`

`ros2 launch` 用于启动 ROS 2 中的启动文件，这些启动文件通常包含了多个节点和参数配置。

### 语法：

```bash
ros2 launch <package> <launch-file>
```

### 示例：

- 启动 `demo_nodes_cpp` 包中的 `add_two_ints.launch.py` 文件：

  ```bash
  ros2 launch demo_nodes_cpp add_two_ints.launch.py
  ```

---

## `ros2 node`

`ros2 node` 用于获取 ROS 2 节点的调试信息。

### 动词说明：

- `info`：显示某个节点的详细信息。
- `list`：列出当前运行的所有节点。

### 示例：

- 显示节点 `/talker` 的信息：

  ```bash
  ros2 node info /talker
  ```

- 列出所有节点：

  ```bash
  ros2 node list
  ```

---

## `ros2 param`

`ros2 param` 用于管理和操作 ROS 2 节点的参数。

### 动词说明：

- `delete`：删除某个节点的参数。
- `describe`：描述某个参数的信息。
- `dump`：导出某个节点的所有参数（YAML 格式）。
- `get`：获取某个节点的指定参数值。
- `list`：列出某个节点的所有参数。
- `load`：加载参数文件到节点。
- `set`：设置某个节点的参数。

### 示例：

- 获取节点 `/talker` 的参数：

  ```bash
  ros2 param get /talker <param-name>
  ```

- 设置节点 `/talker` 的参数：

  ```bash
  ros2 param set /talker <param-name> <param-value>
  ```

- 列出 `/talker` 节点的所有参数：

  ```bash
  ros2 param list
  ```

- 导出 `/talker` 节点的参数到 YAML 文件：

  ```bash
  ros2 param dump /talker
  ```

---

## `ros2 topic`

`ros2 topic` 用于调试 ROS 2 话题，包括查看发布者、订阅者、频率、消息内容等。

### 动词说明：

- `bw`：显示话题的带宽。
- `delay`：显示消息的时间戳延迟。
- `echo`：打印话题的消息内容。
- `find`：查找某个类型的话题。
- `hz`：显示话题的发布频率。
- `info`：显示话题的详细信息。
- `list`：列出所有话题。
- `pub`：发布消息到某个话题。
- `type`：显示话题的消息类型。

### 示例：

- 列出所有话题：

  ```bash
  ros2 topic list
  ```

- 获取 `/chatter` 话题的信息：

  ```bash
  ros2 topic info /chatter
  ```

- 打印 `/chatter` 话题的消息内容：

  ```bash
  ros2 topic echo /chatter
  ```

- 发布消息到 `/chatter` 话题：

  ```bash
  ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello ROS 2 world'"
  ```

