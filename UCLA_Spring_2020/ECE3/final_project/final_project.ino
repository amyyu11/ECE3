#include <ECE3.h>

uint16_t sensorValues[8]; // right -> left, 0 -> 7

int mode = 0; // 0 for ribbon, 1 for straight

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11;
const int right_dir_pin=30; 
const int right_pwm_pin=39;

const int LED_RF = 41;

double error = 0;
double error2 = 0;
double error3 = 0;
double output;
double mapError1 = 0;
double mapError2 = 0;
int dark = 1000;
int donutCount = 0;
int t = 0;
int weights[] = {8,4,2,1};
int kp = 1.5;
int kd = 5;


void setup() {
  ECE3_Init();

  //setup the motors
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);   
  digitalWrite(left_nslp_pin,HIGH);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  Serial.begin(9600);
}


int errorFunction(uint16_t sensorVals[])
{
  int s0 = -weights[0]*sensorVals[0];
  int s1 = -weights[1]*sensorVals[1];
  int s2 = -weights[2]*sensorVals[2];
  int s3 = -weights[3]*sensorVals[3];
  int s4 = weights[3]*sensorVals[4];
  int s5 = weights[2]*sensorVals[5];
  int s6 = weights[1]*sensorVals[6];
  int s7 = weights[0]*sensorVals[7];
  return s0+s1+s2+s3+s4+s5+s6+s7;
}

void donut(){
  leftSpd = 100;
         digitalWrite(right_dir_pin,HIGH);

         analogWrite(left_pwm_pin,leftSpd);
         analogWrite(right_pwm_pin,leftSpd);

         delay(550);

         digitalWrite(right_dir_pin, LOW);

         analogWrite(left_pwm_pin,leftSpd);
         analogWrite(right_pwm_pin,rightSpd);
         delay(50);
    
}

void loop() {
  
  int leftSpd = 65;
  int rightSpd = 65; 

  ECE3_read_IR(sensorValues);
  error = errorFunction(sensorValues);


  mapError1 = map(error, -8000, 8000, -21.5, 21.5);
  output = kp * mapError1 + kd * (mapError1 - error3);


    analogWrite(left_pwm_pin,leftSpd - output);
    analogWrite(right_pwm_pin, rightSpd + output);
 
  if((sensorValues[0] >= dark) && (sensorValues[1] >= dark) && (sensorValues[2] >= dark) && (sensorValues[3] >= dark) 
  && (sensorValues[4] >= dark) && (sensorValues[5] >= dark) && (sensorValues[6] >= dark) && (sensorValues[7] >= dark))
  {
    ECE3_read_IR(sensorValues);
    
    if((mode == 0) && (sensorValues[0] >= dark) && (sensorValues[1] >= dark) && (sensorValues[2] >= dark) && (sensorValues[3] >= dark) 
    && (sensorValues[4] >= dark) && (sensorValues[5] >= dark) && (sensorValues[6] >= dark) && (sensorValues[7] >= dark))
    {
      if(millis() - t >= 300) {
        donutCount++;
        t = millis();
      }
      
     Serial.println(donutCount); 
      
      if (donutCount == 3)
      {
        donut();
      }
      if (donutCount == 6)
      {
        while(true)
        {
           analogWrite(left_pwm_pin,0);
           analogWrite(right_pwm_pin,0);
        }

      }
    }
    else if((mode == 1) && (sensorValues[0] >= dark) && (sensorValues[1] >= dark) && (sensorValues[2] >= dark) && (sensorValues[3] >= dark) 
    && (sensorValues[4] >= dark) && (sensorValues[5] >= dark) && (sensorValues[6] >= dark) && (sensorValues[7] >= dark)){
      donut();
    }
  }

error3 = error2;
error2 = mapError1;

}
