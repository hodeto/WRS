# include <VarSpeedServo.h> //これでサーボの速度変更できるようになる
VarSpeedServo myservo;//サーボモータの名前

const int IO1 = A0; //信号受信用IO1
const int IO2 = A1; //信号受信用IO2
const int IO3 = A2; //信号受信用IO3
const int IO4 = A3; //信号送信用IO4
const int servo = 10; //サーボモータの出力ピン
//Aが前モータBが後ろモータ
const int STBY1 = 6;    // モータードライバ(棚前後)の制御の準備
const int STBY2 = 2;    // モータードライバ(ベルコン)の制御の準備
const int AIN1 = 5;     // 棚(ベルコン)右のDCモーターの制御
const int AIN2 = 4;     // 棚(ベルコン)右のDCモーターの制御
const int BIN1 = 7;     // 棚(ベルコン)左のDCモーターの制御
const int BIN2 = 8;     // 棚(ベルコン)左のDCモーターの制御
const int PWMA = 3;     // 棚(ベルコン)右のDCモーターの回転速度pin
const int PWMB = 11;    // 棚(ベルコン)左のDCモーターの回転速度pin
const int phot1 = A4;     // 棚(ベルコン)右のDCモーターの回転速度pin
const int phot2 = 12;     // 棚(ベルコン)左のDCモーターの回転速度pin
int PA = 250;//右回転速度
int PB = 250;//左回転速度
int conbea =5000;
int IO4time =2000;
int PWMP = 0.5;


void setup() {
   pinMode(STBY1, OUTPUT);
   pinMode(STBY2, OUTPUT);
   pinMode(AIN1, OUTPUT);
   pinMode(AIN2, OUTPUT);
   pinMode(BIN1, OUTPUT);
   pinMode(BIN2, OUTPUT);
   pinMode(PWMA, OUTPUT);
   pinMode(PWMB, OUTPUT);
   pinMode(phot1,INPUT);
   pinMode(phot2,INPUT);
   pinMode(IO1, INPUT);
   pinMode(IO2, INPUT);
   pinMode(IO3, INPUT);
   pinMode(IO4, OUTPUT);
     Serial.begin(9600); 
     myservo.attach(servo);
     analogWrite(PWMA, PA);
     analogWrite(PWMB, PB);
     digitalWrite(STBY1,LOW);
     digitalWrite(STBY2,LOW);
     digitalWrite(IO4,LOW);
     myservo.write(60, 0, true);
     shelf_back();
  
 
}
 
void loop() {
digitalWrite(IO4,LOW);
    
  if(digitalRead(IO3)==HIGH||digitalRead(IO2)==HIGH||digitalRead(IO1)==HIGH){
    delay(500);
  
  if(digitalRead(IO1)==HIGH&&digitalRead(IO2)==HIGH&&digitalRead(IO3)==HIGH){
    Serial.println("IO1 2 3 on");

    conbea_back();
    
    digitalWrite(IO4,HIGH);
    delay(IO4time);
    }
    
  else if(digitalRead(IO1)==HIGH&&digitalRead(IO2)==HIGH){
    Serial.println("IO1 2  on");
 
    PC_rail_up();
    
    digitalWrite(IO4,HIGH);
    delay(IO4time);
    }
    
   else if(digitalRead(IO1)==HIGH&&digitalRead(IO3)==HIGH){
    Serial.println("IO1 3 on");

    conbea_go();
    
    digitalWrite(IO4,HIGH);
    delay(IO4time);
    
    }

   else if(digitalRead(IO2)==HIGH&&digitalRead(IO3)==HIGH){
    Serial.println("IO2 3 on");
    shelf_back();
    
 
    digitalWrite(IO4,HIGH);
    delay(IO4time);
    
    }

   else if(digitalRead(IO1)==HIGH){
    Serial.println("IO1 on");

    PC_rail_down();
    
    digitalWrite(IO4,HIGH);
    delay(IO4time);
    
    }

     else if(digitalRead(IO2)==HIGH){
    Serial.println("IO2 on");
    shelf_go();
    digitalWrite(IO4,HIGH);
    delay(IO4time);
    
    }
     else if(digitalRead(IO3)==HIGH){
    Serial.println("IO3 on");
    digitalWrite(IO4,HIGH);
    delay(IO4time);
    
    }
  }
  else{}
}


void shelf_go(){//棚前進関数
    digitalWrite(STBY1,HIGH);
   while(digitalRead(phot2) == HIGH){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    }
     digitalWrite(STBY1,LOW);
  }

void shelf_back(){//棚後退関数
    digitalWrite(STBY1,HIGH);
    while(digitalRead(phot1) == HIGH){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

    }
     digitalWrite(STBY1,LOW);
  }

void conbea_back(){//ベルコン後ろに
    digitalWrite(STBY2,HIGH);
    
    analogWrite(PWMA, 237);
    analogWrite(PWMB, 250);
    
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    delay(conbea);
     digitalWrite(STBY2,LOW);
  }

void conbea_go(){//ベルコン前に
    digitalWrite(STBY2,HIGH);
    
    analogWrite(PWMA, 250);
    analogWrite(PWMB, 237);
    
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    delay(conbea);
     digitalWrite(STBY2,LOW);
  }

void PC_rail_down(){
   myservo.write(0, 30, true);
  }
void PC_rail_up(){
  myservo.write(60, 30, true);
  }


 
