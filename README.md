# Arduino航模遥控器
基于Arduino Pro Mini的6通道航模遥控器和接收机，成本不到100RMB，实测可达500m遥控距离<br>
教程一：https://www.bilibili.com/read/cv5626316<br>
教程二：https://www.bilibili.com/read/cv5824207<br>
教程三：https://www.bilibili.com/read/cv6080573<br>
视频讲解：https://www.bilibili.com/video/BV1Wk4y1R7N3<br>
拉锯测试：https://www.bilibili.com/video/BV1kk4y1673K<br>
飞行测试：https://www.bilibili.com/video/BV14V411Z7eS<br>
# 遥控器端
6通道ADC采样，4个拨码开关设置通道的正反，1个拨码开关进行混控设置
 * A0~5 -> 六通道电位器输入
 * A6   -> 电压检测
 * 2~5  -> 通道正反开关
 * 6    -> 蜂鸣器
 * 9    -> 混控开关
 * 
 * NRF24L01 | Arduino
 * CE    -> 7
 * CSN   -> 8
 * MOSI  -> 11
 * MISO  -> 12
 * SCK   -> 13
 * IRQ   -> 无连接
 * VCC   -> 小于3.6V
 * GND   -> GND
# 接收机端
6通道PWM输出，SBUS输出
 * PWM输出 -> 引脚2~6、9
 * SBUS输出-> TX0
 * 
 * NRF24L01 | Arduino
 * CE    -> 7
 * CSN   -> 8
 * MOSI  -> 11
 * MISO  -> 12
 * SCK   -> 13
 * IRQ   -> 无连接
 * VCC   -> 小于3.6V
 * GND   -> GND
