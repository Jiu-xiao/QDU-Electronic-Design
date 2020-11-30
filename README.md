# QDU-Electronic-Design
青岛大学2020年电赛校选代码
题目：风扇控制系统
要求：
    1.三种控制模式，对应PWM输出20/30/70
    2.数码管动态显示模式，温度和倒计时
    3.按键控制工作模式，定时运行，停止和温度显示
    4.DS18B20温度检测
    5.LED显示工作模式
方案：使用freeRTOS，设置了六个进程，分别为控制中心，按键检测，温度检测，数码管显示，PWM输出，LED显示。又在原有功能的基础上添加了红外遥控功能。
