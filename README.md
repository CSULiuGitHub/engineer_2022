中南大学FYT Robomaster2022工程车
================================
代码风格
--------------------------
为了兼容以后不规范的代码（后续需要统一）
函数命名 首字母大写下划线分隔Chassis_Process
变量命名 小写字母加下划线front_temp
（后续规范更多，宏，关键字的使用，）

MCU外设资源使用情况
-------------------

1. can

| can\ID | 0x201 | 0x202 | 0x203 | 0x204 |
|:---------:|:--------:|:---------:|:---------:|:--------:|
| can1 | 爪子左|爪子右|前伸|抬升右
| can2 |底盘前左| 底盘前右 |底盘后左|底盘后右|

| can\ID | 0x205 | 0x206 | 0x207 | 0x208 |
|:---------:|:--------:|:---------:|:---------:|:--------:|
| can1 | 抬升左|爪子yaw|图传yaw（6020）
| can2 |下爪子左| 下抓子右 |

2. uart

| uart1 | uart2| uart3 | uart4 | uart5 |
|:---------:|:--------:|:---------:|:---------:|:--------:|
| 遥控器 |调试|  裁判系统|陀螺仪|视觉|

3. tim
4. IO口
5. iic
6. spi


工程控制命令
------------
| 底盘控制 | 
|:---------:|:--------:|:---------:|:---------:|:--------:|
| （1，1）


任务管理
--------


挖坑

临时问题
蓝牙串口发送数据不对
debug断点有问题
遥控器初始化有问题
电机堵转保护程序

深层问题
pid控制算法
类的抽象（不得不说一些结构体就是屎山）