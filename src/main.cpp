#include <Arduino.h>

#define NUM_SERVOS 12

#define RX_PIN 18
#define TX_PIN 19
HardwareSerial BusSerial(2);

#define HDR      0x55
#define CMD_MOVE 0x01
#define CMD_LOAD 0x1F
#define CMD_SERVO_MOTOR_MODE 0x1D  // 29

uint8_t calcCHK(const uint8_t *b) {
  uint16_t s = 0;
  for (uint8_t i = 2; i < b[3] + 2; i++) s += b[i];
  return ~s;
}

void sendPack(uint8_t id, uint8_t cmd, const uint8_t *p, uint8_t n) {
  uint8_t buf[6 + n];
  buf[0] = buf[1] = HDR;
  buf[2] = id; 
  buf[3] = n + 3;
  buf[4] = cmd;
  for (uint8_t i = 0; i < n; i++) buf[5 + i] = p[i];
  buf[5 + n] = calcCHK(buf);
  BusSerial.write(buf, 6 + n);
}

// 設置servo為motor模式
void setMotorMode(uint8_t id, bool enable_motor_mode, int16_t speed = 0) {
  uint8_t mode = enable_motor_mode ? 1 : 0;
  uint8_t p[4] = {
    mode,                    // 參數1: 模式 (0=servo, 1=motor)
    0,                       // 參數2: null
    (uint8_t)(speed & 0xFF), // 參數3: 速度低位
    (uint8_t)(speed >> 8)    // 參數4: 速度高位 (-1000~1000)
  };
  sendPack(id, CMD_SERVO_MOTOR_MODE, p, 4);
}

void enableTorque(uint8_t id) {
  uint8_t on = 1;
  sendPack(id, CMD_LOAD, &on, 1);
}

// 控制馬達速度 (-1000 到 1000)
void setMotorSpeed(uint8_t id, int16_t speed) {
  speed = constrain(speed, -1000, 1000);
  setMotorMode(id, true, speed);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  // 初始化串列通訊
  BusSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // 啟動所有馬達的扭力輸出
  for (uint8_t id = 1; id <= NUM_SERVOS; id++) {
    enableTorque(id);
    delay(20);
  }
  delay(200);
  
  // 設定第3號馬達為馬達模式
  Serial.println("Setting servo ID 3 to motor mode...");
  setMotorMode(3, true, 0);  // 啟用motor模式，初始速度為0
  delay(100);
  Serial.println("Servo ID 3 is now in motor mode");
  
  Serial.println("Setup complete!");
}

void loop() {
  // 範例：讓第3號馬達以不同速度旋轉
  
  Serial.println("Motor forward at speed 500");
  setMotorSpeed(3, 500);   // 正向旋轉
  setMotorSpeed(1, 500);
  delay(3000);
  
  Serial.println("Motor stop");
  setMotorSpeed(3, 0);     // 停止
  setMotorSpeed(1, 0);
  delay(1000);
  
  Serial.println("Motor backward at speed -300");
  setMotorSpeed(3, -300);  // 反向旋轉
  setMotorSpeed(1, -300);
  delay(3000);
  
  Serial.println("Motor stop");
  setMotorSpeed(3, 0);     // 停止
  setMotorSpeed(1, 0);
  delay(1000);
  
  // 你可以透過序列埠監控器輸入指令來控制
  if (Serial.available()) {
    String input = Serial.readString();
    input.trim();
    
    if (input.startsWith("speed")) {
      int speed = input.substring(5).toInt();
      speed = constrain(speed, -1000, 1000);
      Serial.printf("Setting motor speed to: %d\n", speed);
      setMotorSpeed(3, speed);
    }
    else if (input == "stop") {
      Serial.println("Stopping motor");
      setMotorSpeed(3, 0);
    }
  }
}