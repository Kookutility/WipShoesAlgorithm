#include <Arduino_LSM9DS1.h>

#define TRIG_PIN1 2 // 초음파 센서 트리거 핀
#define ECHO_PIN1 3 // 초음파 센서 에코 핀

// 상태 정의
enum MotionState { AIR_STOP, AIR_MOVING, GROUNDED };
MotionState currentState = AIR_STOP;

// IMU 변수
float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
float filteredAccelMag = 0.0;
const float ACCEL_GROUND_THRESHOLD = 1.0; // 정지 판단 기준 (m/s²)
const float GYRO_GROUND_THRESHOLD = 50.0; // 자이로 정지 기준 (deg/s)
const float ALPHA = 0.1; // 저역통과 필터 상수
int groundSampleCount = 0;
const int MIN_GROUND_SAMPLES = 3; // 최소 연속 정지 샘플

// 거리 임계값
const float DISTANCE_THRESHOLD = 10.0; // cm

// 시간 관리
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // 초음파 센서 핀 설정
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);

  // IMU 초기화
  if (!IMU.begin()) {
    Serial.println("IMU 초기화 실패!");
    while (1);
  }
  Serial.println("시스템 초기화 완료");
  Serial.println("상태: AIR_STOP(공중/정지), AIR_MOVING(공중/이동), GROUNDED(착지)");
}

float getDistance() {
  digitalWrite(TRIG_PIN1, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN1, LOW);
  
  long duration = pulseIn(ECHO_PIN1, HIGH, 30000); // 30ms 타임아웃
  if (duration == 0) return 999.0; // 측정 실패 시 큰 값 반환
  
  return (duration * 0.034) / 2; // cm 단위로 변환
}

bool isMoving() {
  // 가속도와 자이로스코프 기반 움직임 판단
  bool accelMoving = abs(filteredAccelMag - 9.81) > ACCEL_GROUND_THRESHOLD;
  bool gyroMoving = abs(gyroY) > GYRO_GROUND_THRESHOLD;
  
  return accelMoving || gyroMoving;
}

String getStateString(MotionState state) {
  switch(state) {
    case AIR_STOP: return "공중/정지";
    case AIR_MOVING: return "공중/이동";
    case GROUNDED: return "착지";
    default: return "알 수 없음";
  }
}

void updateState(float distance, bool moving) {
  MotionState newState = currentState;
  
  switch(currentState) {
    case AIR_STOP:
      if (distance <= DISTANCE_THRESHOLD) {
        newState = GROUNDED;
      } else if (moving) {
        newState = AIR_MOVING;
      }
      break;
      
    case AIR_MOVING:
      if (distance <= DISTANCE_THRESHOLD) {
        newState = GROUNDED;
      } else if (!moving) {
        newState = AIR_STOP;
      }
      break;
      
    case GROUNDED:
      if (distance > DISTANCE_THRESHOLD) {
        if (moving) {
          newState = AIR_MOVING;
        } else {
          newState = AIR_STOP;
        }
      }
      break;
  }
  
  // 상태 변경 시 로그 출력
  if (newState != currentState) {
    Serial.print("상태 변경: ");
    Serial.print(getStateString(currentState));
    Serial.print(" -> ");
    Serial.println(getStateString(newState));
  }
  
  currentState = newState;
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime < 10) return; // 10ms 간격 유지
  
  float deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // 초음파 거리 측정
  float distance = getDistance();

  // IMU 데이터 읽기 및 처리
  bool imuAvailable = IMU.accelerationAvailable() && IMU.gyroscopeAvailable();
  if (imuAvailable) {
    IMU.readAcceleration(accelX, accelY, accelZ);
    IMU.readGyroscope(gyroX, gyroY, gyroZ);

    // 가속도 크기 계산 및 필터링 (m/s²)
    float accelMag = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ) * 9.81;
    filteredAccelMag = ALPHA * accelMag + (1 - ALPHA) * filteredAccelMag;

    // 움직임 상태 판단
    bool moving = isMoving();
    
    // 상태 업데이트
    updateState(distance, moving);

    // 100ms마다 상세 로그 출력
    static unsigned long lastLogTime = 0;
    if (currentTime - lastLogTime >= 100) {
      Serial.print("시간: "); Serial.print(currentTime / 1000.0, 2); Serial.print("초");
      Serial.print(" | 상태: "); Serial.print(getStateString(currentState));
      Serial.print(" | 거리: "); Serial.print(distance, 1); Serial.print("cm");
      Serial.print(" | 움직임: "); Serial.print(moving ? "예" : "아니오");
      Serial.print(" | 필터가속도: "); Serial.print(filteredAccelMag, 2); Serial.print("m/s²");
      Serial.print(" | 자이로Y: "); Serial.print(gyroY, 1); Serial.print("°/s");
      Serial.println();
      lastLogTime = currentTime;
    }
  }
}
