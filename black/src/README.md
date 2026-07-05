# black_mujoco使用说明
## 准备
系统：ubutnu22.04

ros版本：ros2 humble

确保你的设备可以运行ros2

## 依赖
``` bash
pip install mujoco
```
由于依赖或许很多，我不再一一列举，请根据报错自行安装

## 编译
``` bash
cd ~/black_mujoco #进入工作空间

colcon build --packages-select robot_msgs #先编译robot_msgs

colcon build --symlink-install #编译工作空间
```
报错不要慌，安装缺少的依赖即可

## 运行
运行前请确保工作空间已经编译成功
### 启动仿真
``` bash
source install/setup.bash

ros2 launch mujoco_runner mujoco.launch.py rname:=black #rname为机器人名称，你也可以尝试rname:=a1来加载宇树a1机器人
```
没有报错的话即可看到机器人加载在了仿真环境，你也可以打开代码的`src/mujoco_runner/mujoco_runner/config.py`文件将`USE_TERRAIN=False`，改为`USE_TERRAIN=True`，尝试加载地形

### 运行中间件

``` bash
source install/setup.bash

ros2 run midware middleware
```
中间件的作用是控制器和仿真或实机进行数据交互

运行后可以向话题`'/_lowCmd/command'`中发送数据来控制某个关节。
同样的，你可以向话题`'/_lowState/joint'`和`'/_lowState/imu'`中订阅数据，来获取机器人的状态。


代码并不是很复杂，自行阅读