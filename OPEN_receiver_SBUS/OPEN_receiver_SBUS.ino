/* ArduinoProMini-6通道接收机+SBUS输出
 *     by Bilibili 蔡子CaiZi
 * 参考链接https://quadmeup.com/generate-s-bus-with-arduino-in-a-simple-way
S.Bus用11位编码每个RC通道
在内部，通道值映射为：
-100％= 173（相当于PWM伺服信号中的1000）
0％= 992（相当于PWM伺服信号中的1500）
100％= 1811（相当于PMW伺服信号中的2000）
串行端口必须配置为100000bps SERIAL_8E2（8位，偶数，2个停止位）
故障安全状态通过单个标志字节传输
一个S.Bus数据包占用25个字节
每9ms发送一帧（FASSTest 18CH模式）
*/
#define RC_CHANNEL_MIN 1000
#define RC_CHANNEL_MAX 2000
#define SBUS_MIN_OFFSET 173
#define SBUS_MID_OFFSET 992
#define SBUS_MAX_OFFSET 1811
#define SBUS_CHANNEL_NUMBER 16
#define SBUS_PACKET_LENGTH 25
#define SBUS_FRAME_HEADER 0x0f
#define SBUS_FRAME_FOOTER 0x00
#define SBUS_FRAME_FOOTER_V2 0x04
#define SBUS_STATE_FAILSAFE 0x08
#define SBUS_STATE_SIGNALLOSS 0x04
#define SBUS_UPDATE_RATE 4 //ms

#include <SPI.h>
#include <nRF24L01.h> // 安装RF24库
#include <RF24.h>
#include <Servo.h>
Servo ch1;
Servo ch2;
Servo ch3;
Servo ch4;
Servo ch5;
Servo ch6;
struct Signal {    
  byte roll;
  byte pitch;
  byte throttle;  
  byte yaw;
  byte gyr;
  byte pit;
};
Signal data; // 定义一个结构体，用来存储信号
const uint64_t pipeIn = 0xBBBBBBBBB; // 与发射端地址相同
RF24 radio(7, 8);     // SPI通信，引脚对应关系：CE ->7,CSN ->8
void ResetData()
{
  data.roll = 127;    // 横滚通道中心点（254/2 = 127）
  data.pitch = 127;   // 俯仰通道
  data.throttle = 0;  // 信号丢失时，关闭油门
  data.yaw = 127;     // 航向通道
  data.gyr = 0;       //第五通道
  data.pit = 0;       //第六通道
}
void sbusPreparePacket(uint8_t packet[], int channels[], bool isSignalLoss, bool isFailsafe){
    static int output[SBUS_CHANNEL_NUMBER] = {0};//这里一定要16个元素的数组，不然其他通道会干扰
    /*将chanel值1000-2000映射到SBUS协议的173-1811*/
    for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
        output[i] = map(channels[i], RC_CHANNEL_MIN, RC_CHANNEL_MAX, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
    }
    
    uint8_t stateByte = 0x00;
    if (isSignalLoss) {
        stateByte |= SBUS_STATE_SIGNALLOSS; // 丢失信号标志
    }
    if (isFailsafe) {
        stateByte |= SBUS_STATE_FAILSAFE;   // 激活故障安全标志
    }
    packet[0] = SBUS_FRAME_HEADER; //SBUS头，0x0F
    packet[1] = (uint8_t) (output[0] & 0x07FF);
    packet[2] = (uint8_t) ((output[0] & 0x07FF)>>8 | (output[1] & 0x07FF)<<3);
    packet[3] = (uint8_t) ((output[1] & 0x07FF)>>5 | (output[2] & 0x07FF)<<6);
    packet[4] = (uint8_t) ((output[2] & 0x07FF)>>2);
    packet[5] = (uint8_t) ((output[2] & 0x07FF)>>10 | (output[3] & 0x07FF)<<1);
    packet[6] = (uint8_t) ((output[3] & 0x07FF)>>7 | (output[4] & 0x07FF)<<4);
    packet[7] = (uint8_t) ((output[4] & 0x07FF)>>4 | (output[5] & 0x07FF)<<7);
    packet[8] = (uint8_t) ((output[5] & 0x07FF)>>1);
    packet[9] = (uint8_t) ((output[5] & 0x07FF)>>9 | (output[6] & 0x07FF)<<2);
    packet[10] = (uint8_t) ((output[6] & 0x07FF)>>6 | (output[7] & 0x07FF)<<5);
    packet[11] = (uint8_t) ((output[7] & 0x07FF)>>3);
    packet[12] = (uint8_t) ((output[8] & 0x07FF));
    packet[13] = (uint8_t) ((output[8] & 0x07FF)>>8 | (output[9] & 0x07FF)<<3);
    packet[14] = (uint8_t) ((output[9] & 0x07FF)>>5 | (output[10] & 0x07FF)<<6);  
    packet[15] = (uint8_t) ((output[10] & 0x07FF)>>2);
    packet[16] = (uint8_t) ((output[10] & 0x07FF)>>10 | (output[11] & 0x07FF)<<1);
    packet[17] = (uint8_t) ((output[11] & 0x07FF)>>7 | (output[12] & 0x07FF)<<4);
    packet[18] = (uint8_t) ((output[12] & 0x07FF)>>4 | (output[13] & 0x07FF)<<7);
    packet[19] = (uint8_t) ((output[13] & 0x07FF)>>1);
    packet[20] = (uint8_t) ((output[13] & 0x07FF)>>9 | (output[14] & 0x07FF)<<2);
    packet[21] = (uint8_t) ((output[14] & 0x07FF)>>6 | (output[15] & 0x07FF)<<5);
    packet[22] = (uint8_t) ((output[15] & 0x07FF)>>3);
    packet[23] = stateByte;         // 标志位
    packet[24] = SBUS_FRAME_FOOTER; // SBUS结束字节
}

uint8_t sbusPacket[SBUS_PACKET_LENGTH];// 25个字节的数据包
int rcChannels[SBUS_CHANNEL_NUMBER];   // 6通道信号，可以增加
uint32_t sbusTime = 0;
bool signalLoss = false;  // true时表示丢失信号

void setup() {
  //设置PWM信号输出引脚
  ch1.attach(2);
  ch2.attach(3);
  ch3.attach(4);
  ch4.attach(5);
  ch5.attach(6);
  ch6.attach(9);
  //配置NRF24L01模块
  ResetData();
  radio.begin();
  radio.openReadingPipe(1,pipeIn); // 与发射端地址相同
  radio.startListening(); // 接收模式
  pinMode(10,OUTPUT);     // LED推挽输出
  digitalWrite(10,HIGH);
  Serial.begin(100000, SERIAL_8E2); // 将串口波特率设为100000，数据位8，偶校验，停止位2
}
unsigned long lastRecvTime = 0;
void recvData()
{
  while ( radio.available() ) {
    radio.read(&data, sizeof(Signal)); // 接收数据
    lastRecvTime = millis();           // 当前时间ms
  }
}
void loop() {
  recvData();
  unsigned long now = millis();
  if ( now - lastRecvTime > 1000 ) {
    ResetData(); // 两次接收超过1s表示失去信号，输出reset值
    signalLoss = true;
//    Serial.print("无信号");
    digitalWrite(10,HIGH);
  }
  else{
    digitalWrite(10,LOW);
    }
  rcChannels[0] = map(data.roll,     0, 255, 1000, 2000);// 将0~255映射到1000~2000，即(1ms~2ms)/20ms的PWM输出
  rcChannels[1] = map(data.pitch,    0, 255, 1000, 2000);
  rcChannels[2] = map(data.throttle, 0, 255, 1000, 2000);
  rcChannels[3] = map(data.yaw,      0, 255, 1000, 2000);
  rcChannels[4] = map(data.gyr,      0, 255, 1000, 2000);
  rcChannels[5] = map(data.pit,      0, 255, 1000, 2000);
  for (uint8_t i = 6; i < 16; i++) {
        rcChannels[i] = 1500;
  }//未用到的通道全部置中
  uint32_t currentMillis = millis();
  if (currentMillis > sbusTime) {
      sbusPreparePacket(sbusPacket, rcChannels, signalLoss, false); // 6通道数值转换为SBUS数据包
      Serial.write(sbusPacket, SBUS_PACKET_LENGTH); // 将SBUS数据包通过串口TX0输出
      sbusTime = currentMillis + SBUS_UPDATE_RATE;
  }
  ch1.writeMicroseconds(rcChannels[0]);//写入us值，PWM输出
  ch2.writeMicroseconds(rcChannels[1]);
  ch3.writeMicroseconds(rcChannels[2]);
  ch4.writeMicroseconds(rcChannels[3]);
  ch5.writeMicroseconds(rcChannels[4]);
  ch6.writeMicroseconds(rcChannels[5]);
}
