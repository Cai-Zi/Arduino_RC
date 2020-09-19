/* ArduinoProMini-6通道发射器 
 *     by Bilibili 蔡子CaiZi
 *     
 * A0~5 -> 六通道电位器输入
 * A6 -> 电压检测
 * 2~5 -> 通道正反开关
 * 6 -> 蜂鸣器
 * 9 -> 混控开关
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
 */
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
const uint64_t pipeOut = 0xBBBBBBBBB;   //为何这么多B币？与接收器中相同的地址进行通信
RF24 radio(7, 8); // SPI通信，引脚对应关系：CE ->7,CSN ->8
struct Signal {
  byte roll;
  byte pitch;
  byte throttle;
  byte yaw;
  byte gyr;
  byte pit;
};
Signal data;
void ResetData() 
{
  data.roll = 127; // 横滚通道AIL（中心点127）
  data.pitch = 127; // 俯仰通道ELE
  data.throttle = 0; // 信号丢失时，关闭油门THR
  data.yaw = 127; // 航向通道RUD
  data.gyr = 0; //第五通道
  data.pit = 0; //第六通道
}
void setup()
{
  radio.begin();
  radio.openWritingPipe(pipeOut);//pipeOut通信地址
  radio.stopListening(); //发射模式
  ResetData();//初始化6个通道值
  Serial.begin(115200);
  pinMode(2,INPUT);//正反通道开关为数字输入
  pinMode(3,INPUT);
  pinMode(4,INPUT);
  pinMode(5,INPUT);
  pinMode(9,INPUT);//混控开关
  pinMode(6,OUTPUT);//蜂鸣器推挽输出
  if (analogRead(A6)*3.28*3/1023<5){//调整3校准电压检测，5为报警电压
    for(int i=0;i<3;i++){
      digitalWrite(6,HIGH);//蜂鸣器响
      delay(100);
      digitalWrite(6,LOW);
      delay(100);
    }
  }
  else{
    digitalWrite(6,HIGH);//蜂鸣器响
    delay(100);
    digitalWrite(6,LOW);
  }
}

// 将ADC获取的0~1023转换到0~255
int chValue(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);//将val限制在lower~upper范围内
  if ( val < middle )
    val = map(val, lower, middle, 0, 128);
  else
    val = map(val, middle, upper, 128, 255);
  return ( reverse ? 255 - val : val );
}
long x, y, left, right;//混控变量
void loop()
{
//  Serial.print("\t");Serial.print(analogRead(A0));//将数据通过串口输出
//  Serial.print("\t");Serial.print(analogRead(A1));
//  Serial.print("\t");Serial.print(analogRead(A2));
//  Serial.print("\t");Serial.println(analogRead(A3));
  /*三角翼混控*/
  if(digitalRead(9)){//混控开关MIX打开时
    x = chValue( analogRead(A1), 59,  517, 882, digitalRead(2));//副翼舵量
    y = chValue( analogRead(A0), 115, 525, 896, digitalRead(3));//升降舵量
    CtrlRange = 5; // 识别的波动范围，超过范围则切换通道
    if (x>127-CtrlRange and x<127+CtrlRange){ // X轴在小范围内波动，说明没有打副翼
      data.roll = y;          // 此时两个舵机只受升降舵控制
      data.pitch = y;
    }
    else{
      data.roll = 255-x;
      data.pitch = x;
    }
  }
  /*需要对摇杆的最值、中值进行设置*/
  else{
    data.roll     = chValue( analogRead(A0), 59,  517, 882, digitalRead(2));
    data.pitch    = chValue( analogRead(A1), 115, 525, 896, digitalRead(3));
  }
  data.throttle = chValue( analogRead(A2), 145, 522, 920, digitalRead(4));
  data.yaw      = chValue( analogRead(A3), 70,  530, 925, digitalRead(5));
  data.gyr      = chValue( analogRead(A4), 0,   510, 1020, false ); 
  data.pit      = chValue( analogRead(A5), 0,   510, 1020, false );  
  radio.write(&data, sizeof(Signal));//将数据发送出去
//  Serial.print("\t");Serial.print(data.roll);
//  Serial.print("\t");Serial.print(data.pitch);
//  Serial.print("\t");Serial.print(data.throttle);
//  Serial.print("\t");Serial.print(data.yaw);
//  Serial.print("\t");Serial.print(data.gyr);
//  Serial.print("\t");Serial.println(data.pit);
}
