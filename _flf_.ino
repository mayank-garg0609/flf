#define pwmA 3
#define inA1 5
#define inA2 4
#define std 6
#define inB1 7
#define inB2 8
#define pwmB 9
float pins[8] = { A0,A1,A2,A3,A4,A5,A6,A7};
float sensorArray[8];
float sensorValue = 0;
double totalSum = 0, weightedSum = 0;
double distFromCenter[8] = { -300,-150,-50,0,0,50,150,300 };
int threshold = 150;

double maxV[8]={774.00 , 781.00,  782.00,  780.00 , 780.00,  781.00,  784.00 , 781.00  };
double minV[8]={56.00,  76.00  ,75.00 , 69.00,  72.00 , 68.00 , 78.00 , 46.00  };

double Kp = 9; 
double Kd = 0; 
double Ki = 0; 
double P = 0;
double I = 0;
double D = 0;
double lastError = 0; 

double PID = 0;
float error;

void SensorRead() {
  for (int i = 0; i < 8; i++) {
    sensorArray[i] = constrain(map(analogRead(pins[i]), minV[i], maxV[i], 0, 255), 0, 255);
  }

  sensorValue = 0;
  totalSum = 0;
  weightedSum = 0;
  for (int i = 0; i <8; i++) {
    totalSum += sensorArray[i];
    weightedSum += distFromCenter[i] * sensorArray[i];

  error = (weightedSum / totalSum);
}
void calculatePID() {
  P = error;
  I = I + error;
  D = error - lastError;
  PID = P * Kp + I * Ki + D * Kd;
  lastError = error;
  
}
void calibrate() {
  Serial.print("calibrating starting");
  for (int i = 0; i < 2000; i++) {
    for (int j = 0; j <8; j++) {
      int val = analogRead(pins[j]);
      maxV[j] = (val > maxV[j]) ? val : maxV[j];
      minV[j] = (val < minV[j]) ? val : minV[j];
    }
    delay(10);
  }
  Serial.println();
  Serial.print("calibrating end");
  Serial.println();
  Serial.print("max value:");
  Serial.println();
  for(int i=0;i<8;i++){
    Serial.print(maxV[i]);
    Serial.print("  ");
  }
  Serial.println();
  Serial.print("min value:");
  Serial.println();
  for(int i=0;i<8;i++){
    Serial.print(minV[i]);
    Serial.print("  ");
  }
  Serial.println();
}
void motorspeed(int s1,int s2){
  digitalWrite(inA1,LOW);
  digitalWrite(inB1,HIGH);
  digitalWrite(inA2,HIGH);
  digitalWrite(inB2,LOW);
  s1 = constrain(s1, 0, 255);
  s2 = constrain(s2, 0, 255);
  analogWrite(pwmA,s1);
  analogWrite(pwmB,s2);
}
void LineFollow(){
  SensorRead();
  calculatePID();
  int baseSpeed = 200; 
  int speedA = 189  + PID;
  int speedB = 150  - PID;
  motorspeed(speedA,speedB);
}

void setup(){
  Serial.begin(9600);
  pinMode(pwmA,OUTPUT);
  pinMode(pwmB,OUTPUT);
  pinMode(inA1,OUTPUT);
  pinMode(inA2,OUTPUT);
  pinMode(std,OUTPUT);
  pinMode(inB1,OUTPUT);
  pinMode(inB2,OUTPUT);
  digitalWrite(std , HIGH);
  
  
  
  calibrate();
  
  
}
void loop(){ 
//   SensorRead();
 LineFollow();
//  for (int i = 0; i < 8; i++) {
//   Serial.print(sensorArray[i]);
//   Serial.print(" ");
//  }
//  Serial.println();
//  Serial.println(error);
//  delay(200);
//  motorspeed(189 , 150);

}