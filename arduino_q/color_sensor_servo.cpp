#include "color_sensor_servo.h"
#include "tca9548a.h"

// 舵机对象
Servo sensorServo;

// 舵机参数
const int MIN_ANGLE = 0;
const int MAX_ANGLE = 270;     // ✅ 270度
const int STEP_DELAY = 8;
const int ANGLE_STEP = 1;

// ✅ 角度通道映射表（保持不变）
AngleChannelMap angleMaps[] = {
  {0,    0, 5},
  {47,   1, 6},
  {90,   2, 7},
  {130,  3, 8},
  {175,  4, 9}
};
const int ANGLE_MAP_COUNT = sizeof(angleMaps) / sizeof(angleMaps[0]);

// 颜色二维码映射表
ColorQrMap colorQrMap[] = {
  {"Red",    'r'},
  {"Yellow", 'y'},
  {"Blue",   'b'},
  {"Green",  'g'},
  {"White",  'w'}
};
const int COLOR_QR_COUNT = sizeof(colorQrMap) / sizeof(colorQrMap[0]);

// 颜色通道映射数组
const char* channelToColor[10] = {"Unknown", "Unknown", "Unknown", "Unknown", "Unknown", 
                                   "Unknown", "Unknown", "Unknown", "Unknown", "Unknown"};
int colorToChannels[5][2];
int colorChannelCount[5];
bool isMappingDone = false;

// TCA通道定义
#define TCA_COLOR_SENSOR1_CH 0
#define TCA_COLOR_SENSOR2_CH 1

// ✅ 270度舵机控制函数
void servo270Write(int angle) {
  // 限制角度范围
  angle = constrain(angle, SERVO_270_MIN_ANGLE, SERVO_270_MAX_ANGLE);
  
  // 将角度映射到脉宽 (0-270度 → 500-2500微秒)
  int pulseWidth = map(angle, 
                       SERVO_270_MIN_ANGLE, SERVO_270_MAX_ANGLE,
                       SERVO_270_MIN_PULSE, SERVO_270_MAX_PULSE);
  
  // 发送脉宽信号
  sensorServo.writeMicroseconds(pulseWidth);
  
  // 调试输出
  Serial.print(F("Servo270: "));
  Serial.print(angle);
  Serial.print(F("° → Pulse: "));
  Serial.print(pulseWidth);
  Serial.println(F("μs"));
}

// ✅ 初始化：使用270度函数
void sensorServoInit() {
  sensorServo.attach(SENSOR_SERVO_PIN);
  servo270Write(0);  // ✅ 使用270度函数
  delay(500);
}

// ✅ 推球：全部使用270度函数
void pushball() {
  servo270Write(0);
  delay(500);
  
  servo270Write(270);
  delay(500);
  
  servo270Write(0);
  delay(100);
  
  // 从0度扫到270度
  for (int angle = MIN_ANGLE; angle <= MAX_ANGLE; angle += ANGLE_STEP) {
    servo270Write(angle);
    delay(STEP_DELAY);
  }
  delay(100);
  
  // 从270度返回0度
  for (int angle = MAX_ANGLE; angle >= MIN_ANGLE; angle -= ANGLE_STEP) {
    servo270Write(angle);
    delay(STEP_DELAY);
  }
  delay(100);
}

bool colorSensorPing(int sensorNum) {
  tcaSelect(sensorNum == 1 ? TCA_COLOR_SENSOR1_CH : TCA_COLOR_SENSOR2_CH);
  
  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(CMD_PING);
  Wire.endTransmission(false);

  Wire.requestFrom(COLOR_SENSOR_ADDR, 1);
  if (Wire.available()) {
    return Wire.read() == PING_RESPONSE;
  }
  return false;
}

bool colorSensorInit() {
  bool sensor1Ready = false;
  bool sensor2Ready = false;
  
  for (int i = 0; i < 5; i++) {
    if (!sensor1Ready) {
      sensor1Ready = colorSensorPing(1);
      delay(SENSOR_READ_DELAY);
    }
    if (!sensor2Ready) {
      sensor2Ready = colorSensorPing(2);
      delay(SENSOR_READ_DELAY);
    }
    
    if (sensor1Ready && sensor2Ready) break;
    delay(100);
  }
  
  Serial.print(F("Sensor 1 ready: ")); 
  Serial.println(sensor1Ready ? F("Yes") : F("No"));
  Serial.print(F("Sensor 2 ready: ")); 
  Serial.println(sensor2Ready ? F("Yes") : F("No"));
  
  return sensor1Ready && sensor2Ready;
}

void readRGB(int rgb[3], int sensorNum) {
  tcaSelect(sensorNum == 1 ? TCA_COLOR_SENSOR1_CH : TCA_COLOR_SENSOR2_CH);
  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(CMD_READ_RGB);
  Wire.endTransmission(false);

  Wire.requestFrom(COLOR_SENSOR_ADDR, 3);
  int i = 0;
  while (Wire.available() && i < 3) {
    rgb[i++] = Wire.read();
  }
  delay(10);
}

void readHSL(int hsl[3], int sensorNum) {
  tcaSelect(sensorNum == 1 ? TCA_COLOR_SENSOR1_CH : TCA_COLOR_SENSOR2_CH);
  Wire.beginTransmission(COLOR_SENSOR_ADDR);
  Wire.write(CMD_READ_HSL);
  Wire.endTransmission(false);

  Wire.requestFrom(COLOR_SENSOR_ADDR, 3);
  int i = 0;
  while (Wire.available() && i < 3) {
    hsl[i++] = Wire.read();
  }
  delay(10);
}

const char* detectColorSensor1() {
  int rgb[3];
  readRGB(rgb, 1);
  
  int r = rgb[0], g = rgb[1], b = rgb[2];
  
  if (r > 200 && g < 100 && b < 100) return "Red";
  if (r > 200 && g > 150 && b < 100) return "Yellow";
  if (r < 100 && g < 100 && b > 150) return "Blue";
  if (r < 150 && g > 150 && b < 100) return "Green";
  if (r > 200 && g > 200 && b > 200) return "White";
  
  return "Unknown";
}

const char* detectColorSensor2() {
  int rgb[3];
  readRGB(rgb, 2);
  
  int r = rgb[0], g = rgb[1], b = rgb[2];
  
  if (r > 200 && g < 100 && b < 100) return "Red";
  if (r > 200 && g > 150 && b < 100) return "Yellow";
  if (r < 100 && g < 100 && b > 150) return "Blue";
  if (r < 150 && g > 150 && b < 100) return "Green";
  if (r > 200 && g > 200 && b > 200) return "White";
  
  return "Unknown";
}

const char* detectColor(int sensorNum) {
  if (sensorNum == 1) {
    return detectColorSensor1();
  } else {
    return detectColorSensor2();
  }
}

void initDynamicMapping() {
  for (int c = 0; c < 5; c++) {
    colorChannelCount[c] = 0;
    colorToChannels[c][0] = -1;
    colorToChannels[c][1] = -1;
  }
}

void updateColorToChannels(const char* color, int channel) {
  channelToColor[channel] = color;
  
  for (int c = 0; c < 5; c++) {
    if (strcmp(colorQrMap[c].colorName, color) == 0) {
      if (colorChannelCount[c] < 2) {
        colorToChannels[c][colorChannelCount[c]] = channel;
        colorChannelCount[c]++;
      }
      break;
    }
  }
}

void printDynamicMapping() {
  Serial.println(F("\n=== Color-Channel Mapping ==="));
  Serial.println(F("Channel → Color:"));
  for (int ch = 0; ch < 10; ch++) {
    Serial.print(F("  Channel "));
    Serial.print(ch);
    Serial.print(F(": "));
    Serial.println(channelToColor[ch]);
  }
  
  Serial.println(F("\nColor → Channels:"));
  for (int c = 0; c < 5; c++) {
    Serial.print(F("  "));
    Serial.print(colorQrMap[c].colorName);
    Serial.print(F(": "));
    for (int i = 0; i < 2; i++) {
      if (colorToChannels[c][i] != -1) {
        Serial.print(colorToChannels[c][i]);
        Serial.print(F(" "));
      }
    }
    Serial.println();
  }
  Serial.println(F("============================"));
}

int getChannelsByColor(const char* color, int channels[]) {
  int count = 0;
  for (int c = 0; c < 5; c++) {
    if (strcmp(colorQrMap[c].colorName, color) == 0) {
      for (int i = 0; i < 2; i++) {
        if (colorToChannels[c][i] != -1) {
          channels[count++] = colorToChannels[c][i];
        }
      }
      break;
    }
  }
  return count;
}

// ✅ 颜色映射：全部使用270度函数
void autoMapColors() {
  servo270Write(0);  // ✅ 使用270度函数
  delay(500);
  
  for (int i = 0; i < ANGLE_MAP_COUNT; i++) {
    int angle = angleMaps[i].angle;
    int channel1 = angleMaps[i].channel1;
    int channel2 = angleMaps[i].channel2;
    
    Serial.print(F("\nMoving to angle: "));
    Serial.println(angle);
    
    servo270Write(angle);  // ✅ 使用270度函数
    delay(500);
    
    const char* color1 = detectColorSensor1();
    updateColorToChannels(color1, channel1);
    Serial.print(F("Sensor 1 detected "));
    Serial.print(color1);
    Serial.print(F(" for channel "));
    Serial.println(channel1);
    
    delay(SENSOR_READ_DELAY);
    
    const char* color2 = detectColorSensor2();
    updateColorToChannels(color2, channel2);
    Serial.print(F("Sensor 2 detected "));
    Serial.print(color2);
    Serial.print(F(" for channel "));
    Serial.println(channel2);
  }
  
  servo270Write(0);  // ✅ 使用270度函数
  delay(500);
}