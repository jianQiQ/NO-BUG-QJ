#define FRAME_HEAD 0xAA
#define FRAME_TAIL 0x55


// 接收缓冲区
uint8_t rxBuffer2[4];
uint8_t rxIndex2 = 0;
bool frameReceived2 = false;
unsigned long lastSendTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Arduino UART Master Ready");
  Serial.println("Sending: data=0");
  sendData2(0);
  lastSendTime = millis();
}  // 修复：添加右花括号

void loop() {
  // 每1秒发送数据0到STM32
  if (millis() - lastSendTime >= 1000) {
    sendData2(0);  // Arduino发送0
    lastSendTime = millis();
  }
  
  // 接收STM32返回的数据
  while (Serial.available() > 0) {
    uint8_t inByte2 = Serial.read();
    
    // 状态机接收
    if (rxIndex2 == 0 && inByte2 == FRAME_HEAD) {
      // 检测到帧头
      rxBuffer2[rxIndex2++] = inByte2;
    } else if (rxIndex2 > 0 && rxIndex2 < 4) {
      rxBuffer2[rxIndex2++] = inByte2;

      // 接收完整帧（4字节）
      if (rxIndex2 == 4) {
        if (inByte2 == FRAME_TAIL) {
          frameReceived2 = true;
        } else {
          // 接收错误，重置
          rxIndex2 = 0;
          Serial.println("Frame error: wrong tail");
        }
      }
    }
  }
  
  // 处理接收到的完整帧
  if (frameReceived2) {
    // 校验帧头和帧尾
    if (rxBuffer2[0] == FRAME_HEAD && rxBuffer2[3] == FRAME_TAIL) {  
      uint8_t data4 = rxBuffer2[1];       // 数据在第2字节
      uint8_t checksum = rxBuffer2[2];    // 校验在第3字节

      // 校验数据（去掉长度字段）
      uint8_t calc_checksum = FRAME_HEAD + data4;  
      if (checksum == calc_checksum) {
        Serial.print("Received from STM32: data4=");
        Serial.print(data4);
        Serial.println(" (valid)");

        // 如果数据4为 1，则点亮 LED
        if (data4 == 1) {
          
        } else {
          
        }
      } else {
        Serial.println("Checksum error!");
      }
    } else {
      Serial.println("Frame validation error!");
    }
    
    // 重置接收状态
    rxIndex2 = 0;
    frameReceived2 = false;
  }
  
  delay(10);
}  // 修复：只保留一个右花括号

// 发送数据到STM32
void sendData2(uint8_t data4) {
  uint8_t frame[4];                    // 修复：改为4字节
  frame[0] = FRAME_HEAD;
  frame[1] = data4;                    // 数据
  frame[2] = FRAME_HEAD + data4;       // 校验（去掉长度）
  frame[3] = FRAME_TAIL;

  Serial.write(frame, 4);
  Serial.print("Sent to STM32: data=");
  Serial.println(data4);
}