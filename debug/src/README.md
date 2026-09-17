# player说明
player通过订阅 图像，IMU 以及 机器人头部的关节角度 这三个信息，对数据进行处理，形成控制机器人的策略，通过 `BodyTask` 以及 `HeadTask` 发布出去，从而实现机器人的控制
## 图像数据
```cpp
auto image = imageSubscriber->GetImage().clone();
```
图像数据通过上面的接口获得，其格式为OpenCV中的Mat格式，RGB三通道。

## IMU数据
```cpp
auto imuData = imuSubscriber->GetData();
```
IMU数据通过上面的接口获得，参赛队伍主要需要关心的是其中的yaw变量，用于表示机器人当前的朝向，具体角度多少代表那个方向请自行测试

## 头部关节角数据
```cpp
auto headAngle = headSubscriber->GetData();
```
头部关节数据通过上面的接口获得，参赛队伍主要需要关心的是其中的yaw和pitch变量，用于表示机器人头部的方向和俯仰

## BodyTask
BodyTask用于控制机器的身体运动，其定义和说明如下：
```
int32 type
string actname
float64 step
float64 lateral
float64 turn
int32 count

int32 TASK_WALK=1
int32 TASK_ACT=2
```
type为TASK_WALK时表示当前为行走任务，与行走相关的变量有：
- `step`表示前进量，单位为米，正数前进，负数后退；
- `lateral`表示横移量，单位为米，正数左移，负数右移；
- `turn`表示转动的角度，单位为度，正数左转，负数右转；
- `count`表示行走步数，一般取值2。

type为TASK_ACT时表示当前为执行特殊动作，与执行动作相关的变量为`actname`，其含义为需要执行的动作的名称，参赛队伍需要使用到的只有左脚或者右脚踢球动作和初始的准备动作，名称分别为：`"left_kick", "right_kick", "ready"`

## HeadTask
HeadTask用于控制机器人头部的角度，其定义和说明如下：
```
int32 mode
float64 yaw
float64 pitch
```
其中 `mode` 目前没有使用，`yaw` 表示头部的方向，`pitch` 表示头部的俯仰，单位均为度，具体的正负对应关系请自行测试。

## 处理后的图像如何查看
代码中已经提供接口，通过提供的接口发布处理后的图像后，打开一个新的终端，运行rqt工具， 在Plugins->Visualization下选择Image View，之后在界面中选择格式为xxxx/result/image的话题，即可查看处理好的图像，其中xxxx表示机器人名称

## gameData

gameData可用于获取比赛情况，允许调用的数据有：
- `red_score` 和 `blue_score`：两队得分
- `state`：目前比赛状态，包括 `STATE_INIT`，`STATE_READY`，`STATE_PLAY`，`STATE_PAUSE`，`STATE_END`。

不允许调用其它接口，尤其是不允许调用函数修改比分或状态等。一旦发现立即取消成绩。

## 定位数据

location 可用于获取机器人的定位，但定位精度为一米，其定义和说明如下：
```
float32 x
float32 z
```

x 轴垂直于中场线/球门线，z 轴则平行于中场线/球门线。
