# black 实机部署底层代码

本仓库为西安交通大学 RoboCon 四足组 black 机器人的实机部署底层代码，主要包含 ROS 2 工作区中的机器人底层通信、实机运行节点、消息定义、手柄控制、机器人描述以及相关调试工具。

## 目录概览

- `black/`：black 机器人实机部署工作区。
- `blackW/`：blackW 相关工作区，包含轮组/连杆测试等扩展调试内容。
- `black/src/`、`blackW/src/`：主要源码目录。
- `black/tools/`：实机调试与数据监控脚本。

## 使用说明

在对应工作区目录下构建：

```bash
colcon build
```

构建完成后加载环境：

```bash
source install/setup.bash
```

随后根据具体任务启动对应 ROS 2 节点或调试脚本。

## 说明

仓库根目录的 `.gitignore` 已忽略 `build/`、`install/`、`log/` 等构建产物和运行日志，提交代码时建议仅保留源码、配置和必要文档。
