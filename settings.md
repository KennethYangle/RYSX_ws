## 使用

直接在settings.json修改，或创建新的文件，在main.py和fake_pub.py中修改文件路径。

## 参数说明

1. `MODE`: `RealFlight` 或者 `Simulation`。实际飞行时设置为`RealFlight`，有遥控器指示飞机。仿真实验时使用`Simulation`，键盘和游戏手柄控制飞机。
2. `car_velocity`: 汽车匀速运动速度。使用中把这个参数作为飞行器限速，飞行器最大速度为`3*car_velocity`。
3. `follow_mode`: `0`或`1`。`0`表示使用预设距离，跟踪距离为参数`follow_distance`。`1`表示使用摆放位置，跟踪距离为摆放位置计算得到。
4. `FLIGHT_H`: 飞行高度。表示飞行器相对于home点高度。
5. `Utils`: 控制器相关参数。其中`saftyz`表示z方向限速；`cam_offset`表示稳定时激光照设点与靶标中心像素偏移量。
6. `Simulation`: 仿真噪声相关参数。`GPS_offset`是人为加的GPS偏差，GPS数据会与模型真实位置产生一定偏移，而深度仍是真值。`cam_lose_cnt`是相机丢失时间，计数大于该值不再有图像数据，800是一个较好的值。