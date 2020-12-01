# QDU-Electronic-Design  
青岛大学2020年电赛校选代码  
题目：风扇控制系统  
要求：  
    1.三种控制模式，对应PWM输出20/30/70  
    2.数码管动态显示模式，温度和倒计时  
    3.按键控制工作模式，定时运行，停止和温度显示  
    4.DS18B20温度检测  
    5.LED显示工作模式  
方案：使用freeRTOS，设置了六个进程，分别为控制中心，按键检测，温度检测，数码管显示，PWM输出，LED显示。  
# 电路部分  
原理图与PCB绘制都使用AD完成，放在/PCB目录下，与正点原子mini板通过排母连接，电源通过排线连接，同时提供额外的供电接口，使用跳线切换。  
# 备注  
    1.使用了正点原子mini板上的PB2接口作为输入，因为PB2在板子上用作BOOT1,通过跳线接地,所以不使用跳线帽，而是通过一个100k电阻下拉。  
    2.原理图与实际PCB切换了PC9与PB14,还有其他差别，以PCB为准。  
    3.使用板载DS18B20,温度检测与延时代码均来自网络。  
    4.PCB上的开关封装有问题，实际直接使用导线短接  

