#include <A4988.h>
#include <BasicStepperDriver.h>
#include <DRV8825.h>
#include <DRV8834.h>
#include <DRV8880.h>
#include <MultiDriver.h>
#include <SyncDriver.h>

#include <A4988.h>
#include <BasicStepperDriver.h>
#include <DRV8825.h>
#include <DRV8834.h>
#include <DRV8880.h>
#include <MultiDriver.h>
#include <SyncDriver.h>

#include <Arduino.h> 
#include "BasicStepperDriver.h" 
#include "MultiDriver.h" 
#include "SyncDriver.h" 
#include <math.h> 
// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step 
#define MOTOR_STEPS 200   //モータのハードウェア的特性 
// Target RPM for cruise speed 
#define RPM 440           //目標到達速度 
// Acceleration and deceleration values are always in FULL steps / s^2 
//XY軸加減速度 
#define MOTOR_ACCEL_X 10000   //最大6000 
#define MOTOR_DECEL_X 12000 
#define MOTOR_ACCEL_Y 6000   //最大8000 
#define MOTOR_DECEL_Y 9000 
//Z軸加減速度 
#define MOTOR_ACCEL_Z 16000   //Zは遅くあれ 
#define MOTOR_DECEL_Z 16000 
#define DeadTime 10   //各動作待ち時間ms 
// Microstepping mode. If you hardwired it to save pins, set to the same value here. 
#define MICROSTEPS 16   //モータのハードウェア的特性 
#define DIR_X 5 
#define STEP_X 2 
#define DIR_Y 6 
#define STEP_Y 3 
#define DIR_Z 7 
#define STEP_Z 4 
#include "A4988.h" 
A4988 stepperX(MOTOR_STEPS, DIR_X, STEP_X); 
A4988 stepperY(MOTOR_STEPS, DIR_Y, STEP_Y); 
A4988 stepperZ(MOTOR_STEPS, DIR_Z, STEP_Z); 
SyncDriver controller(stepperX, stepperY, stepperZ); 
#include <Servo.h> 
Servo servo1;   //Servoオブジェクトの宣言 
Servo servo2; 
Servo servo3; 
Servo servo4; 
Servo servo5; 
Servo servo6;
Servo servo7; 
Servo servo8; 
Servo servo9;
#define servo1_pin 36 
#define servo2_pin 38 
#define servo3_pin 40 
#define servo4_pin 42 
#define servo5_pin 44  
#define servo6_pin 46 
#define servo7_pin 48 
#define servo8_pin 50 
#define servo9_pin 52 
#define Toggle_Down 22 
#define Toggle_Up 24 
#define Enter_SW 26 
#define PD 24   //ピッチ円直径 
#define Lead 8   //リードスクリュー送り量 8mm/回転 
int cal_dis_to_r(int target){   //目標距離から回転角へ変換(XY軸) 
  return (int)(( (float)target / ((float)PD * M_PI)) * 360); 
} 

int cal_Z(int target){   //目標距離から回転角へ変換(Z軸専用) 
  return (int)(( (float)target / ((float)Lead)) * 360); 
} 

//ここからピックプレースやる
int Only_place(int startX, int startY, int pickX, int pickY, int 
placeX, int placeY, int endX, int endY, int g1, int g2, int g3, int g4, int g5, int g6, int g7, int g8, int g9, int zdowm){  //開始座標、ピッキング座標、プレース座標、行動終了座標の順、 開始座標からの相対座標系 ,各グリッパ0で閉じる1で開く,ｚのピック移動　1有効　0無効
  //X+左、Y+前進、Z+下がる 
  //cal_dis_to_r(目的座標 - 現在座標) 
  //スタート座標から、ピッキング座標へ 
  controller.rotate(cal_dis_to_r(pickX - startX), 0, 0); 
  controller.rotate(0, cal_dis_to_r(pickY - startY), 0); 
  delay(DeadTime); 
  //下降して掴む 
  //ピッキング座標から、プレース座標へ
 

  //X軸シフト、前進 
  controller.rotate(cal_dis_to_r(placeX - pickX), 0, 0); 
  controller.rotate(0, cal_dis_to_r(placeY - pickY), 0); 
  delay(DeadTime); 
  //下降して離す 
  controller.rotate(0, 0, cal_Z(58-0)); 
  servo9.write(144*g9);   //グリッパ開く 
  servo8.write(144*g8);   //グリッパ開く 
  servo7.write(144*g7);   //グリッパ開く 
  servo6.write(144*g6);   //グリッパ開く
  servo5.write(144*g5);   //グリッパ開く 
  servo4.write(144*g4);   //グリッパ開く 
  servo3.write(144*g3);   //グリッパ開く 
  servo2.write(144*g2);   //グリッパ開く 
  servo1.write(144*g1);   //グリッパ開く 
  delay(300); 
  //上昇退避する 
  controller.rotate(0, 0, cal_Z(0-58)); 
  delay(DeadTime); 
  //プレース座標から、終了座標へ 
  //Y軸引き抜き、X軸シフト 
  controller.rotate(0, cal_dis_to_r(endY - placeY), 0); 
  controller.rotate(cal_dis_to_r(endX - placeX), 0, 0); 
  delay(DeadTime); 
} 

 
 
int Auto_pick_and_place(int startX, int startY, int pickX, int pickY, int 
placeX, int placeY, int endX, int endY, int g1, int g2, int g3, int g4, int g5, int g6, int g7, int g8, int g9, int zdowm){  //開始座標、ピッキング座標、プレース座標、行動終了座標の順、 開始座標からの相対座標系 ,各グリッパ0で閉じる1で開く,ｚのピック移動　1有効　0無効
  //X+左、Y+前進、Z+下がる 
  //cal_dis_to_r(目的座標 - 現在座標) 
  //スタート座標から、ピッキング座標へ 
  controller.rotate(cal_dis_to_r(pickX - startX), 0, 0); 
  controller.rotate(0, cal_dis_to_r(pickY - startY), 0); 
  delay(DeadTime); 
  //下降して掴む 
  controller.rotate(0, 0, cal_Z(170)); 
  servo1.write(0);   //グリッパ閉じる 
  servo2.write(0);   //グリッパ閉じる 
  servo3.write(0);   //グリッパ閉じる 
  servo4.write(0);   //グリッパ閉じる 
  servo5.write(0);   //グリッパ閉じる 
  servo6.write(0);   //グリッパ閉じる 
  servo7.write(0);   //グリッパ閉じる 
  servo8.write(0);   //グリッパ閉じる 
  servo9.write(0);   //グリッパ閉じる 
  
  delay(300); 
  //controller.rotate(0, cal_dis_to_r(pickY - (pickY-15)), 0);  //トラクション戻す 
  //上昇退避する 
  controller.rotate(0, 0, cal_Z(-170)); 
  delay(DeadTime); 
  //ピッキング座標から、プレース座標へ 

 
  //X軸シフト、前進 
  controller.rotate(cal_dis_to_r(placeX - pickX), 0, 0); 
  controller.rotate(0, cal_dis_to_r(placeY - pickY), 0); 
  delay(DeadTime); 
 
  //下降して離す 
  controller.rotate(0, 0, cal_Z(58-0)); 
  servo9.write(144*g9);   //グリッパ開く 
  servo8.write(144*g8);   //グリッパ開く 
  servo7.write(144*g7);   //グリッパ開く 
  servo6.write(144*g6);   //グリッパ開く
  servo5.write(144*g5);   //グリッパ開く 
  servo4.write(144*g4);   //グリッパ開く 
  servo3.write(144*g3);   //グリッパ開く 
  servo2.write(144*g2);   //グリッパ開く 
  servo1.write(144*g1);   //グリッパ開く 
  delay(300); 
 
  //上昇退避する 
  controller.rotate(0, 0, cal_Z(0-58)); 
  delay(DeadTime); 
 
  //プレース座標から、終了座標へ 
  //Y軸引き抜き、X軸シフト 
  controller.rotate(0, cal_dis_to_r(endY - placeY), 0); 
  controller.rotate(cal_dis_to_r(endX - placeX), 0, 0); 
  delay(DeadTime); 
} 
 
int Re_Auto_pick_and_place(int startX, int startY, int pickX, int pickY, int 
placeX, int placeY, int endX, int endY, int g1, int g2, int g3, int g4, int g5, int g6, int g7, int g8, int g9, int zdowm){   //開始座標、ピッキング座標、プレース座標、行動終了座標の順、 開始座標からの相対座標系 
  //X+左、Y+前進、Z+下がる 
  //cal_dis_to_r(目的座標 - 現在座標) 
 
  //スタート座標から、ピッキング座標へ 
  controller.rotate(cal_dis_to_r(pickX - startX), 0, 0); 
  controller.rotate(0, cal_dis_to_r(pickY - startY), 0); 
  delay(DeadTime); 
 
  //下降して掴む 
  controller.rotate(0, 0, cal_Z(72-0)); 
  servo1.write(0);   //グリッパ閉じる  
  servo2.write(0);   //グリッパ閉じる 
  servo3.write(0);   //グリッパ閉じる 
  servo4.write(0);   //グリッパ閉じる 
  servo5.write(0);   //グリッパ閉じる 
  servo6.write(0);   //グリッパ閉じる  
  servo7.write(0);   //グリッパ閉じる 
  servo8.write(0);   //グリッパ閉じる 
  servo9.write(0);   //グリッパ閉じる 
  
  delay(500); 
  //上昇退避する 
  controller.rotate(0, cal_dis_to_r(pickY - (pickY-10)), 0);   //トラクション戻す 

  controller.rotate(0, 0, cal_Z(0-72)); 
  delay(DeadTime); 
  //ピッキング座標から、プレース座標へ 
  //前進、X軸シフト 
  controller.rotate(0, cal_dis_to_r(placeY - pickY), 0); 
  controller.rotate(cal_dis_to_r(placeX - pickX), 0, 0); 
  delay(DeadTime); 
  //下降して離す 
  controller.rotate(0, 0, cal_Z(160-0)); 
  servo9.write(144*g9);   //グリッパ開く 
  servo8.write(144*g8);   //グリッパ開く 
  servo7.write(144*g7);   //グリッパ開く 
  servo6.write(144*g6);   //グリッパ開く
  servo5.write(144*g5);   //グリッパ開く 
  servo4.write(144*g4);   //グリッパ開く 
  servo3.write(144*g3);   //グリッパ開く 
  servo2.write(144*g2);   //グリッパ開く 
  servo1.write(144*g1);   //グリッパ開く 
  delay(300); 
  //上昇退避する 
  controller.rotate(0, 0, cal_Z(0-160)); 
  delay(DeadTime); 
  //プレース座標から、終了座標へ 
  //X軸シフト、Y軸引き抜き 
  controller.rotate(cal_dis_to_r(endX - placeX), 0, 0); 
  controller.rotate(0, cal_dis_to_r(endY - placeY), 0); 
  delay(DeadTime); 
} 
#define MAIN_MODE   //有効でメイン処理 
//#define DEBUG_MODE  //有効でデバックモード処理 
//#define HAND_DEBUG_ON   ////サーボグリッパ指先取付、取り外し用 デバックモード ,有効化でデバックモード 
void setup() { 
  Serial.begin(115200); 
  pinMode(8, OUTPUT);   //8:ENAピン 1でモーター動作、 0で機能停止 
  digitalWrite(8,0); 
  pinMode(Toggle_Up, INPUT_PULLUP); 
  pinMode(Toggle_Down, INPUT_PULLUP); 
  pinMode(Enter_SW, INPUT_PULLUP); 
  stepperX.begin(RPM, MICROSTEPS); 
  stepperX.enable(); 
  stepperY.begin(RPM, MICROSTEPS); 
  stepperY.enable(); 
  stepperZ.begin(RPM, MICROSTEPS); 
  stepperZ.enable(); 
  /* 
*Set LINEAR_SPEED (accelerated) profile.
付録 50 
    */ 
  stepperX.setSpeedProfile(stepperX.LINEAR_SPEED, MOTOR_ACCEL_X, 
MOTOR_DECEL_X); 
  stepperY.setSpeedProfile(stepperY.LINEAR_SPEED, MOTOR_ACCEL_Y, 
MOTOR_DECEL_Y); 
  stepperZ.setSpeedProfile(stepperZ.LINEAR_SPEED, MOTOR_ACCEL_Z, 
MOTOR_DECEL_Z); 
  servo1.attach(servo1_pin, 585, 2400);   //サーボモータ設定 
  servo2.attach(servo2_pin, 585, 2400);   //サーボモータ設定 
  servo3.attach(servo3_pin, 585, 2400);   //サーボモータ設定 
  servo4.attach(servo4_pin, 585, 2400);   //サーボモータ設定 
  servo5.attach(servo5_pin, 585, 2400);   //サーボモータ設定 
  servo6.attach(servo6_pin, 585, 2400);   //サーボモータ設定 
  servo7.attach(servo7_pin, 585, 2400);   //サーボモータ設定 
  servo8.attach(servo8_pin, 585, 2400);   //サーボモータ設定 
  servo9.attach(servo9_pin, 585, 2400);   //サーボモータ設定 
#ifdef HAND_DEBUG_ON   //サーボグリッパ指先取付、取り外し用 この時間中に取り付けてね 
  servo1.write(162);   //グリッパ開く 
  servo2.write(162);   //グリッパ開く 
  servo3.write(162);   //グリッパ開く 
  servo4.write(162);   //グリッパ開く 
  servo5.write(162);   //グリッパ開く 
  servo2.write(162);   //グリッパ開く 
  servo3.write(162);   //グリッパ開く 
  servo4.write(162);   //グリッパ開く 
  servo5.write(162);   //グリッパ開く 
  delay(3000); 
#endif 
  //ハンド初期姿勢 
  servo1.write(144);   //グリッパ開く 
  servo2.write(144);   //グリッパ開く 
  servo3.write(144);   //グリッパ開く 
  servo4.write(144);   //グリッパ開く 
  servo5.write(144);   //グリッパ開く 
  servo6.write(144);   //グリッパ開く 
  servo7.write(144);   //グリッパ開く 
  servo8.write(144);   //グリッパ開く 
  servo9.write(144);   //グリッパ開く 
  delay(1000); 
  Serial.println("START"); 
} 
void loop() { 
while(digitalRead(Enter_SW) == 0){  //実行スイッチ押下後の処理式 
#ifdef MAIN_MODE   //大会プログラム            
  if(digitalRead(Toggle_Up)==0){ 
    delay(3000); 
    //サーモン左から 
    Auto_pick_and_place(0, 0, 0, 0, 470, 440, 685, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1 ,0); 
    Only_place(680, 0, 680, 0, 680, 440, 100, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1 ,0); //開始座標XY(この系における原点座標)、ピッキング座標XY、プレース座標XY、行動終了座標XYの順 
    Auto_pick_and_place(100, 0, 100, 0, 470, 220, 590, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1 ,0); 
    Only_place(590, 0, 590, 0, 590, 440, 520, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1 ,0);
    //梅
    Auto_pick_and_place(520, 0, 520, 0, 370, 440, 410, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0);
    Auto_pick_and_place(410, 0, 410, 0, 270, 440, 310, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1 ,0);  
    //ツナ
    Auto_pick_and_place(310, 0, 310, 0, 150, 490, 210, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0);  //開始座標XY(この系における原点座標)、ピッキング座標XY、プレース座標XY、行動終了座標XYの順 
    Auto_pick_and_place(210, 0, 210, 0, 150, 120, 40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0);
    Only_place(40, 0, 40, 0, 40, 440, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1 ,0);
    //Auto_pick_and_place(215, 0, 215, 0, 15, 490, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0); 

  } 
 
  else if(digitalRead(Toggle_Down)==0){ 
    delay(3000); 
    //梅 
    Re_Auto_pick_and_place(0, 0, 0, 0, 640, 490, 430, 80,0,0,0,0,0,0,0,0,0,0);
    //開始座標XY(この系における原点座標)、ピッキング座標XY、プレース座標XY、行動終了座標XYの順 
    Re_Auto_pick_and_place(0, 0, 0, 0, 201, -420, 108, 0,0,0,0,0,0,0,0,0,0,0); 
    //ツナ右から 
    Re_Auto_pick_and_place(0, 0, 0, 0, -119, -420, 108, 0,0,0,0,0,0,0,0,0,0,0);   //開始座標XY(この系における原点座標)、ピッキング座標XY、プレース座標XY、行動終了座標XYの順 
    Re_Auto_pick_and_place(0, 0, 0, 0, -121, -420, 108, 0,0,0,0,0,0,0,0,0,0,0); 
    //梅 
    Re_Auto_pick_and_place(0, 0, 0, 0, -441, -420, 108, 0,0,0,0,0,0,0,0,0,0,0);   //開始座標XY(この系における原点座標)、ピッキング座標XY、プレース座標XY、行動終了座標XYの順 
    Re_Auto_pick_and_place(0, 0, 0, 0, -655, -420, 0, -430,0,0,0,0,0,0,0,0,0,0); 
  } 
  else{ 
    Serial.print("ERROR"); 
  } 
#endif 
#ifdef DEBUG_MODE 
デバックモードの処理式 
  controller.rotate(cal_dis_to_r(600-0), 0, 0); 
  delay(100); 
  controller.rotate(0, cal_dis_to_r(300-0), 0); 
  delay(100); 
  controller.rotate(0, cal_dis_to_r(0-300), 0); 
  delay(100); 
 controller.rotate(cal_dis_to_r(0-600), 0, 0); 
  delay(100); 
  controller.rotate(0, 0, cal_Z(100-0)); 
  delay(100); 
  
 controller.rotate(0, 0, cal_Z(0-100)); 
  delay(100); 
 controller.rotate(0, 0, cal_Z(160-0)); 
  servo1.write(0);   //グリッパ閉じる 
  servo2.write(0);   //グリッパ閉じる 
  servo3.write(0);   //グリッパ閉じる 
  servo4.write(0);   //グリッパ閉じる 
  servo5.write(0);   //グリッパ閉じる 
  servo6.write(0);   //グリッパ閉じる 
  servo7.write(0);   //グリッパ閉じる 
  servo8.write(0);   //グリッパ閉じる 
  servo9.write(0);   //グリッパ閉じる 
  delay(500); 
 
  controller.rotate(0, 0, cal_Z(0-160)); 
  servo1.write(140);   //グリッパ開く 
  servo2.write(140);   //グリッパ開く 
  servo3.write(140);   //グリッパ開く  
  servo4.write(140);   //グリッパ開く 
  servo5.write(140);   //グリッパ開く 
  servo6.write(140);   //グリッパ開く 
  servo7.write(140);   //グリッパ開く 
  servo8.write(140);   //グリッパ開く  
  servo9.write(140);   //グリッパ開く 
  delay(500); 
#endif 
} 
}
