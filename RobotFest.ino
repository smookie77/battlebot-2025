#include <PS4Controller.h>
#define R_SPEED 26
#define L_SPEED 25
#define L_BRK 16
#define L_DIR 4
#define R_BRK 15
#define R_DIR 13
#define MAX_SPEED 128
int leftX;
int leftY;
int brake;
int throttle;
void notify(){
  brake=PS4.L2Value();
  throttle=PS4.R2Value();
  leftX=PS4.LStickX();
  leftY=PS4.LStickY();
}
void onConnect(){
  Serial.println("Connected!");
}
void drive(){
  int dir=0;
  if(leftY>=10){
    dir=-1;
    digitalWrite(R_DIR,LOW);
    digitalWrite(L_DIR,HIGH);
  }else if(leftY<=-10){
    dir=1;
    digitalWrite(R_DIR,HIGH);
    digitalWrite(L_DIR,LOW);
  }
  throttle= map(throttle,0,255,0,MAX_SPEED);
  int speed;
  if(throttle>10){
    speed=throttle;
  }
  int lSpeed=0,rSpeed=0;
  if(leftX>=0){
    lSpeed=speed;
    rSpeed=speed*abs(leftY)/127;
    if(rSpeed>255){
      rSpeed=255;
    }
  }else{
    rSpeed=speed;
    lSpeed=speed*abs(leftY)/127;
    if(lSpeed>255){
      lSpeed=255;
    }
  }

  if(brake>10){
    analogWrite(L_BRK,brake);
    analogWrite(R_BRK,brake);
    // Serial.println("BREAKING!");
  }else{
    analogWrite(L_BRK,0);
    analogWrite(R_BRK,0);
  }
  dacWrite(L_SPEED,lSpeed);
  dacWrite(R_SPEED,rSpeed);
  // Serial.print("BR= ");
  // Serial.print(brake);
  // Serial.print(" LS= ");
  // Serial.print(lSpeed);
  // Serial.print(" RS= ");
  // Serial.print(rSpeed);
  // Serial.print(" Th= ");
  // Serial.println(throttle);
  delay(10);
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  PS4.begin("3C:E9:0E:7F:25:A6");
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  pinMode(L_BRK,OUTPUT);
  pinMode(R_BRK,OUTPUT);
  pinMode(L_DIR,OUTPUT);
  pinMode(R_DIR,OUTPUT);
  Serial.println("Ready.");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!PS4.isConnected()) {
    return;
  }
  drive();
}
