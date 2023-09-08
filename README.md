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
![304e427ac67732827476ff940afe7b0](https://github.com/CN-Hacker/ATETestProgram/assets/143678738/16808c8f-3ef4-4b22-a6ad-2181915a176e)



