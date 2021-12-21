#include <Servo.h>     // [20213066] 서보모터 라이브러리 사용

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
//#define PIN_LED 9          // [20200000] LED 핀 정의
#define PIN_SERVO 10       // [20213069] servo 핀 정의
#define PIN_IR A0      // [20213077] analong 핀 정의 

// Framework setting
#define _DIST_TARGET 255  // [20213080] 멈추려는 목표거리 정의
#define _DIST_MIN 100 // [20213058] 측정 최소거리, 플로터 범위
#define _DIST_MAX 410    // [20213065] 측정 최대거리,플로터 범위

#define A 70  // 센서 최소
#define B 250  // 센서 최대

// Distance sensor
#define _ITERM_MAX 60
// #define _DIST_ALPHA 0.3

#define DELAY_MICROS  1550 // 필터에 넣을 샘플값을 측정하는 딜레이(고정값!)
#define EMA_ALPHA 0.3    

// Servo range
#define _DUTY_MIN 1200    // [20213060] 서보 duty 최소값
#define _DUTY_NEU 1500    // [20213064] 서보 duty 중간값
#define _DUTY_MAX 1800    // [20213079] 서보 duty 최댓값

// Servo speed control
#define _SERVO_ANGLE 100       // [20213062] 서보 각도
#define _SERVO_SPEED 250     // [20213075] 서보의 속도
  
// Event periods
#define _INTERVAL_DIST 10   // [20213058] 거리측정 주기
#define _INTERVAL_SERVO 10    // [20213055] 서보 조정 주기  
#define _INTERVAL_SERIAL 200  // [20213058] 시리얼 제어 주기

// PID parameters
#define _KP 1.0   // [20213058]비례이득
#define _KD 70.0     // 미분이득
#define _KI 0.005     // 적분이득

//////////////////////
// global variables //
//////////////////////

// Servo instance 
Servo myservo;    // [20213068] 서보 변수명을 myservo로 선언

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;    // [20213077] 측정값, ema필터 적용값

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial; // [20213072] 거리, 서보, 시리얼 측정 여부

// Servo speed control
int duty_chg_per_interval; // [20213058]한 주기 당 제어할 최대 duty값
int duty_target, duty_curr; // [20213055] 목표 pulse주기값, 현재 pulse주기값

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

float ema_dist=0;            // EMA 필터에 사용할 변수
float filtered_dist;       // 최종 측정된 거리값을 넣을 변수. loop()안에 filtered_dist = filtered_ir_distance(); 형태로 사용하면 됨.
float samples_num = 3;     // 스파이크 제거를 위한 부분필터에 샘플을 몇개 측정할 것인지. 3개로 충분함! 가능하면 수정하지 말 것.

float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}

void setup() {
  // initialize GPIO pins for LED and attach servo 
  //pinMode(PIN_LED,OUTPUT);
  myservo.attach(PIN_SERVO);

  // move servo to neutral position
  duty_target = duty_curr = _DUTY_NEU; // [20213075] 초기화
  myservo.writeMicroseconds(duty_curr); // [20213078] 서보 중립위치

  // initialize serial port
  Serial.begin(57600); // [20213058]

// initialize global variables 
dist_target = _DIST_TARGET;
dist_raw = dist_ema = ir_distance();
error_curr = error_prev = dist_target - dist_raw;

last_sampling_time_dist = 0; // [20213078] last_sampling_time_dist 초기화
last_sampling_time_servo = 0; // [20213055]
last_sampling_time_serial = 0; // [20213080]
/*
pterm = _KP * error_curr;
dterm =  _KD * (error_curr - error_prev);
iterm += _KI * error_curr; 
*/
event_dist = event_servo = event_serial = false;

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / ((float)_SERVO_ANGLE) * _INTERVAL_SERVO / 1000; // [20213078] 한 주기 당 제어할 최대 duty값 초기화
}
  


void loop() {
/////////////////////
// Event generator //
/////////////////////
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
//[20213080]


////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
     event_dist = false; //[20213078]
  // get a distance reading from the distance sensor
      dist_raw = filtered_ir_distance(); // [20213055]
      dist_raw = 100 + 300.0 / (B - A) * (dist_raw - A);
      if (dist_raw > _DIST_MAX) dist_raw = _DIST_MAX;
      
  // PID control logic
      error_curr = dist_target - dist_raw; // [20213075]
      pterm = _KP * error_curr; // [20213078]
      dterm = _KD * (error_curr - error_prev);  //[20213077]
      iterm += _KI * error_curr;

      if(abs(iterm) > _ITERM_MAX){ iterm = iterm/2;}
      if(iterm > _ITERM_MAX){ iterm = _ITERM_MAX;}
      if(iterm < -_ITERM_MAX){ iterm = -_ITERM_MAX;}
        
      control = pterm + dterm + iterm; // [20213078]

  // duty_target = f(duty_neutral, control)
    //duty_target = _DUTY_NEU + control; // [20213055]
    duty_target = _DUTY_NEU *(control/300+1); // 이런 방법??

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
  if(duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;
  if(duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;

   error_prev = error_curr;  // error update
  }
  
  if(event_servo) {
    event_servo = false; // [20213078]
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    // update servo position
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
       duty_curr -= duty_chg_per_interval;
       if(duty_curr < duty_target) duty_curr = duty_target;
    } // [20213075]

  }
    // update servo position
    myservo.writeMicroseconds(duty_curr); // [20213058]
    event_servo = false;
  

  if(event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
 // [20213071] 
// Min :측정 최소거리, Max : 최대거리, Low : 목표구역 최소거리, dist_target :기준이 되는 거리, High : 목표구역 최대거리 값을 시리얼 모니터에 표시

  }
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
} // [20213075]

float ir_distance_filtered(void){ // return value unit: mm
  return ir_distance(); // for now, just use ir_distance() without noise filter.
}
