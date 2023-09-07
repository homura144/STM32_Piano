# STM32_Piano

## 简介

清华大学“计算机硬件技术基础”课程大作业, 基于STM32G431RBTX实现电子琴

## 功能

- STM32用PWM信号控制蜂鸣器播放、弹奏
- STM32用MIDI信号控制PC的对应软件播放、弹奏
- LED和数码管显示状态
- 通过按键控制弹奏、播放、功能切换
- PC通过USB串口与STM32通信,远程控制弹奏、播放、功能切换

## 编译

- 使用STM32CubeMX配置工程,配置文件为`STM32_Piano.ioc`
- 使用STM32CubeIDE编译
- CLion用户可以使用`CMakeLists.txt`构建项目

## 目录结构

| 文件夹     | 说明            |
|---------|---------------|
| core    | 源代码           |
| drivers | 驱动文件          |
| docs    | 课程文档，对项目的详细说明 |
| images  | 单片机引脚图        |
| config  | OpenOCD配置文件   |
