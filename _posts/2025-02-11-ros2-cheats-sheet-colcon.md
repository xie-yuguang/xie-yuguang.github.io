---
title: ROS2 Cheats Sheet - Colcon
description: >-
  ros2 colcon的命令参数使用示例
author: xie
date: 2025-02-11 15:02:00 +0800
categories: [Basic, ROS2]
tags: [ROS2]
pin: true
media_subpath: '/posts/20250211'
---

# **ROS2 Cheats Sheet - colcon**

## 1. colcon 简介**

`colcon` 是一个用于构建、测试和使用多个 ROS 2 软件包的命令行工具。它可以自动管理软件包之间的依赖关系，并设置合适的环境。

所有 `colcon` 命令都以 `colcon` 作为前缀，后跟命令和（可能的）参数。

要获取某个 `colcon` 命令的帮助信息，可以使用：

```bash
colcon <command> --help
```

此外，`colcon` 提供命令自动补全。例如：

```bash
colcon <command> [tab][tab]
```

更多信息请参考 [colcon 官方文档](https://colcon.readthedocs.io/en/released/index.html)。

---

## 2. 环境变量**

| 变量                          | 说明                                                              |
| --------------------------- | --------------------------------------------------------------- |
| `CMAKE_COMMAND`             | CMake 可执行文件的完整路径                                                |
| `COLCON_ALL_SHELLS`         | 启用所有 shell 扩展的标志                                                |
| `COLCON_COMPLETION_LOGFILE` | 设置自动补全日志文件                                                      |
| `COLCON_DEFAULTS_FILE`      | YAML 配置文件路径（默认 `$COLCON_HOME/defaults.yaml`）                    |
| `COLCON_HOME`               | 配置目录（默认 `~/.colcon`）                                            |
| `COLCON_LOG_LEVEL`          | 设置日志级别（`debug=10`，`info=20`，`warn=30`，`error=40`，`critical=50`） |
| `COLCON_LOG_PATH`           | 日志目录（默认 `$COLCON_HOME/log`）                                     |

---

## 3. 常用 colcon 命令**

### \*\*3.1. 构建 (`colcon build`)

构建 ROS 2 工作区内的软件包。

#### **示例**

- **构建整个工作区**
  ```bash
  colcon build
  ```
- **构建指定软件包（不包含依赖）**
  ```bash
  colcon build --packages-select demo_nodes_cpp
  ```
- **构建两个软件包（包含依赖）并使用符号链接**
  ```bash
  colcon build --packages-up-to demo_nodes_cpp action_tutorials \
               --symlink-install --event-handlers console_direct+
  ```

#### **常用选项**

| 选项                    | 说明                 |
| --------------------- | ------------------ |
| `--symlink-install`   | 使用符号链接代替复制文件       |
| `--continue-on-error` | 遇到错误时继续构建其他软件包     |
| `--cmake-args`        | 传递参数给 CMake        |
| `--cmake-clean-cache` | 删除 CMake 缓存并强制重新配置 |

---

### \*\*3.2. 列出信息 (`colcon list`)

列出 ROS 2 工作区内的软件包信息。

#### **示例**

- **列出所有软件包**
  ```bash
  colcon list
  ```
- **按拓扑顺序列出软件包名称**
  ```bash
  colcon list --names-only --topological-order --packages-up-to demo_nodes_cpp
  ```

#### **扩展命令**

| 命令                        | 说明          |
| ------------------------- | ----------- |
| `colcon extension-points` | 列出所有扩展点     |
| `colcon extensions`       | 列出所有扩展      |
| `colcon info`             | 列出所有软件包的元信息 |

---

### \*\*3.3. 运行测试 (`colcon test`)

对软件包进行测试。

#### **示例**

- **测试整个工作区**
  ```bash
  colcon test
  ```
- **测试单个软件包（不包含依赖）**
  ```bash
  colcon test --packages-select demo_nodes_cpp
  ```
- **测试包含依赖的软件包**
  ```bash
  colcon test --packages-above demo_nodes_py
  ```
- **测试两个软件包，并在终端显示测试结果**
  ```bash
  colcon test --packages-up-to demo_nodes_cpp demo_nodes_py \
              --event-handlers console_direct+
  ```
- **运行 **``** 并生成覆盖率报告**
  ```bash
  colcon test --packages-select demo_nodes_cpp \
              --event-handlers console_direct+ \
              --pytest-args --cov=sros2
  ```

#### **查看测试结果 (**``**)**

- **显示所有测试结果（包括成功的测试）**
  ```bash
  colcon test-result --all
  ```

---

### \*\*3.4. 版本检查 (`colcon version-check`)

对比本地软件包版本与 PyPI 版本。

#### **示例**

```bash
colcon version-check
```

---

## **4. 其他 colcon 选项**

| 选项                                   | 说明                      |
| ------------------------------------ | ----------------------- |
| `--event-handlers console_direct+`   | 直接在终端显示输出               |
| `--event-handlers console_cohesion+` | 在软件包构建完成后显示输出           |
| `--packages-select <pkg>`            | 仅构建指定软件包                |
| `--packages-up-to <pkg>`             | 构建指定软件包及其依赖             |
| `--packages-above <pkg>`             | 构建指定软件包及其反向依赖           |
| `--packages-skip <pkg>`              | 跳过指定软件包的构建              |
| `--cmake-clean-first`                | 先执行 `cmake clean` 然后再构建 |
| `--cmake-force-configure`            | 强制执行 CMake 配置步骤         |

---

## **5. 参考链接**

- [colcon 官方文档](https://colcon.readthedocs.io/en/released/index.html)
- [ROS 2 官方文档](https://docs.ros.org/)
- [ROS 2 Tutorials](https://docs.ros.org/en/foxy/Tutorials.html)



