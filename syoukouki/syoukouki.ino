#include "pcon_driver.h"

// Define the maximum possible message size we'll ever need.
// The read response is the largest at 37 bytes.
#define MAX_BUFFER_SIZE 128

// Buffers for sending and receiving data.
// Declared globally to avoid stack overflow issues in loops.
uint8_t command_buffer[MAX_BUFFER_SIZE];
uint8_t response_buffer[MAX_BUFFER_SIZE];
const int IO2 = A1; //信号受信用IO2
const int IO3 = A2; //信号受信用IO3
const int IO4 = A3; //信号送信用IO4
// Instantiate the driver, passing the hardware serial port you are using.
// For Uno, this is typically just 'Serial'.
// For boards like Mega with multiple serial ports, you could use Serial1, Serial2, etc.
// constexpr int PCON_RX_PIN = 17;
// constexpr int PCON_TX_PIN = 18;
// it requires 230400 baud, so SoftwareSerial is not appropriate but HardwareSerial is required
PconDriver pcon(Serial);

void print_buffer(const uint8_t* buffer, size_t size) {
  Serial.print("  TX Data -> [ ");
  for (size_t i = 0; i < size; i++) {
    if (buffer[i] < 0x10) {
      Serial.print("0"); // 見やすくするために0を補完
    }
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println("]");
}

void setup() {
  // デバッグ用にArduinoのシリアルモニタを開始します。
  // PCONコントローラとの通信を開始します。
  // このボーレートはコントローラの設定と一致させる必要があります。
  long pcon_baud_rate = 230400;
  pcon.begin(pcon_baud_rate); // これでpconSerialが初期化されます。
  delay(1000);
  
  PconStatus status;
  int expected_response_size = 37;
  
  pcon.createReadRegisterMessage(command_buffer, MAX_BUFFER_SIZE, expected_response_size);
  
  if (pcon.sendMessage(command_buffer, 8, response_buffer, expected_response_size)) {
    pcon.parseMessage(response_buffer, expected_response_size, status);
  } else {
  }

  

  // --- Example 1: アラームをリセット ---
  pcon.createResetAlarmMessage(command_buffer, MAX_BUFFER_SIZE);
  // 書き込みコマンドは8バイトの応答（コマンドのエコー）を期待します。
  if (pcon.sendMessage(command_buffer, 8, response_buffer, 8)) {
  } else {
  }
  
  delay(500);

  // --- Example 2: サーボをONにする ---
  pcon.createServoOnMessage(command_buffer, MAX_BUFFER_SIZE);
  if (pcon.sendMessage(command_buffer, 8, response_buffer, 8)) {
  } else {
  }
  delay(1000); // サーボの準備が整うのを待ちます。
  
  DeviceStatus device_status = pcon.parseDeviceStatusOne(status.device_status_one);

  if(!device_status.is_home_pose){
    pcon.createGoHomeMessage(command_buffer, MAX_BUFFER_SIZE);
    if(pcon.sendMessage(command_buffer, 8, response_buffer, 8)){

    }
  }

  delay(10000);
pinMode(IO2, INPUT);
pinMode(IO3, INPUT);
pinMode(IO4, OUTPUT);
}


void loop() {
  
  // --- Example 3: 2秒ごとにコントローラのステータスを読み取り、表示する ---
  PconStatus status;
  int expected_response_size = 37;
  
  pcon.createReadRegisterMessage(command_buffer, MAX_BUFFER_SIZE, expected_response_size);
  
  if (pcon.sendMessage(command_buffer, 8, response_buffer, expected_response_size)) {
    pcon.parseMessage(response_buffer, expected_response_size, status);
    Serial.println("Parsed Message.");

  } else {
  }

  delay(2000);


  if(digitalRead(IO3)==HIGH){
  // --- Example 4: 位置400.0 mmへ移動 ---
  pcon.createMotorMoveMessage(command_buffer, MAX_BUFFER_SIZE, 400.0, 0.1, 50.0, 0.3);
  if (pcon.sendMessage(command_buffer, 23, response_buffer, 8)) {
    print_buffer(command_buffer, 23);
  } else {
  }
  digitalWrite(IO4,HIGH);
  delay(2000);
  digitalWrite(IO4,LOW);
  delay(100);
  }

   if(digitalRead(IO2)==HIGH){
  // --- Example 4: 位置0.0 mmへ移動 ---
  pcon.createMotorMoveMessage(command_buffer, MAX_BUFFER_SIZE, 0.0, 0.1, 50.0, 0.3);
  if (pcon.sendMessage(command_buffer, 23, response_buffer, 8)) {
    print_buffer(command_buffer, 23);
  } else {
  }
  digitalWrite(IO4,HIGH);
  delay(2000);
  digitalWrite(IO4,LOW);
  delay(100);
  }
 
}
