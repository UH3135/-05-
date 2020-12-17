#include <Servo.h>

#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

#define _DIST_TARGET 255
#define _DIST_MIN 150
#define _DIST_MAX 350

#define _DIST_ALPHA 0.36

#define _DUTY_MIN 1220    
#define _DUTY_NEU 1412
#define _DUTY_MAX 1600   

#define _SERVO_ANGLE 30
#define _SERVO_SPEED 750

#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100

#define LENGTH 30
#define k_LENGTH 8

#define _KP 1.05
#define _KD 17.5
#define _KI 0.015

// global variables
Servo myservo; 

float dist_target; 
float dist_raw;
float dist_list[LENGTH], sum, dist_ema, alpha;

unsigned long last_sampling_time_dist, last_sampling_time_servo,
last_sampling_time_serial; 

bool event_dist, event_servo, event_serial;

int duty_chg_per_interval; 
int duty_target, duty_curr;

float error_curr, error_prev, control, pterm, dterm, iterm;

int a, b;
int correction_dist, iter;



void setup() {
  a = 82.25;
  b = 245;
  correction_dist = 0;
  iter = 0; sum = 0;

  alpha = _DIST_ALPHA;
  
  pinMode(PIN_LED,OUTPUT);
  myservo.attach(PIN_SERVO); 

  duty_target = duty_curr = _DUTY_NEU;
  myservo.writeMicroseconds(duty_curr); 
  last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial = 0;

  dist_raw, dist_ema = _DIST_MIN;
  
  Serial.begin(57600); 

  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_SERVO / 1000);
}


  
void loop() {
    
    unsigned long time_curr = millis(); 
    
    if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
    }
    if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
    }
    if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
    }


  if(event_dist) {
    event_dist = false; 
    
    dist_raw = ir_distance_filtered();

    error_curr = _DIST_TARGET - dist_raw; 
    pterm = error_curr * _KP ;
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    control = pterm + dterm + iterm;  // 제어량 계산
    duty_target = _DUTY_NEU + control;

    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; 

    error_prev = error_curr;

    last_sampling_time_dist = millis(); // [3133] 마지막 dist event 처리 시각 기록
  }
  
  if(event_servo) {
    event_servo = false; // [3133]
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr){
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target)duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    } 

    // update servo position
    myservo.writeMicroseconds((int)duty_curr);
    last_sampling_time_servo = millis(); // [3133] 마지막 servo event 처리 시각 기록
  }




  if(event_serial) {
    event_serial = false; // [3133]
    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(", T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(", -G:245,+G:265,m:0,M:800");
    
    last_sampling_time_serial = millis(); // [3133] 마지막 serial event 처리 시각 기록
  }
}

float ir_distance(void){ // return value unit: mm 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float ir_distance_filtered() {
  sum = 0;
  iter = 0;
  while (iter < LENGTH)
  {
    dist_list[iter] = 150 + 350.0 / (b - a) * (ir_distance() - a);
    sum += dist_list[iter];
    iter++;
  }

  for (int i = 0; i < LENGTH-1; i++){
    for (int j = i+1; j < LENGTH; j++){
      if (dist_list[i] > dist_list[j]) {
        float tmp = dist_list[i];
        dist_list[i] = dist_list[j];
        dist_list[j] = tmp;
      }
    }
  }
  
  for (int i = 0; i < k_LENGTH; i++) {
    sum -= dist_list[i];
  }
  for (int i = 1; i <= k_LENGTH; i++) {
    sum -= dist_list[LENGTH-i];
  }

  float dist_cali = sum/(LENGTH-2*k_LENGTH);
  dist_ema = alpha*dist_cali + (1-alpha)*dist_ema;
  
  return dist_ema;
}

 
