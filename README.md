ATETestProgram

一.20230908：

1.版本说明：
      
      当前ATE测试版本为验证Sensor版本阶段。

2.TestProgram文件说明
  4个文件为HYCE06-01—ATE机台，测试HSC02版Sensor所用测试程序与上位机工程；

  （1）AlgorithmForHSC02A文件定义各个图像算法测试项的数据转换；

  （2）imageAlgorithmn文件实现检测图像缺陷相关功能；

  （3）HSC02ACode文件为实现调用机台硬件资源对Sensor供电和图像算法回传数值转换功能；

3.HSC02A为上位机工程，主要包含配置电测相关指标的测试条件和上下限和测暗图、亮图项写入的Register，及其实现人机交互的界面。
  
  （1）当前可测试项：
  
![image](https://github.com/CN-Hacker/ATETestProgram/assets/143678738/bd20e5ad-6502-4a54-90bb-9ba504cdec20)

      电性能：OS_VSS、OS_VDD、PowerShort、IIL/IIH、IDD_Active
      通讯交互项：IIC
      暗图项：Dark_DPCOn、Dark_DPCOff、DarkCurrent、TempNoise、DarkFPN
      亮图项：Light、FWC、PRNU、CG、Sensitivity
  
  （2）Configure文件集
  
  ![image](https://github.com/CN-Hacker/ATETestProgram/assets/143678738/6f44f1b0-cf5e-4caa-8253-1f7521d32099)


  （3）配合code设置匹配xml文件集
  
![image](https://github.com/CN-Hacker/ATETestProgram/assets/143678738/c534382e-0d93-42af-ac5a-8993df90e4e8)
