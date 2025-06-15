#include <Arduino_LSM9DS1.h>

#define TRIG_PIN1 2 // 초음파 센서
#define ECHO_PIN1 3

// 상태 정의
enum MotionState { AIR_STOP, AIR_MOVING, GROUNDED };
MotionState currentState = AIR_STOP;
MotionState previousState = AIR_STOP;

// IMU 변수
float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
float filteredAccelMag = 0.0;
float velocity = 0.0; // 현재 속도 (m/s)
float accumulatedVelocity = 0.0; // 공중/이동 상태에서 누적된 속도
float maxVelocity = 0.0; // 최대 속도 추적
float oppositeVelocity = 0.0; // 착지 시 반대 속도

// 센서 임계값
const float ACCEL_GROUND_THRESHOLD = 1.5; // 정지 판단 기준 (m/s²)
const float GYRO_GROUND_THRESHOLD = 30.0; // 자이로 정지 기준 (deg/s)
const float DISTANCE_THRESHOLD = 10.0; // 착지 판단 거리 (cm)
const float ALPHA = 0.2; // 저역통과 필터 상수
const float VELOCITY_THRESHOLD = 0.05; // 최소 속도 임계값 (m/s)

// 드리프트 보정 변수
float accelBiasX = 0.0; // X축 가속도 바이어스
const int CALIBRATION_SAMPLES = 100; // 캘리브레이션 샘플 수
bool isCalibrated = false;

// 시간 관리
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // 핀 설정
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);

  // IMU 초기화
  if (!IMU.begin()) {
    Serial.println("IMU 초기화 실패!");
    while (1);
  }
  
  Serial.println("제자리걸음 신발 속도 측정 시스템 초기화 완료");
  Serial.println("캘리브레이션 시작... 신발을 정지 상태로 유지하세요.");
  
  // IMU 캘리브레이션
  calibrateIMU();
}

void calibrateIMU() {
  float sumAccelX = 0.0;
  
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(accelX, accelY, accelZ);
      sumAccelX += accelX;
      delay(10);
    }
  }
  
  accelBiasX = sumAccelX / CALIBRATION_SAMPLES;
  isCalibrated = true;
  
  Serial.print("캘리브레이션 완료! X축 바이어스: ");
  Serial.println(accelBiasX, 4);
}

float getDistance() {
  digitalWrite(TRIG_PIN1, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN1, LOW);
  
  long duration = pulseIn(ECHO_PIN1, HIGH, 30000);
  if (duration == 0) return 999.0;
  
  return (duration * 0.034) / 2;
}

bool isMoving() {
  bool accelMoving = abs(filteredAccelMag - 9.81) > ACCEL_GROUND_THRESHOLD;
  bool gyroMoving = abs(gyroY) > GYRO_GROUND_THRESHOLD;
  return accelMoving || gyroMoving;
}

void calculateVelocity(float deltaTime) {
  // 바이어스 보정된 전진 방향 가속도 (X축)
  float correctedAccelX = accelX - accelBiasX;
  float forwardAccel = correctedAccelX * 9.81; // g를 m/s²로 변환
  
  // 속도 적분 (사다리꼴 적분법 사용)
  static float previousAccel = 0.0;
  velocity += (forwardAccel + previousAccel) * 0.5 * deltaTime;
  previousAccel = forwardAccel;
  
  // 속도 감쇠 (공기 저항 및 마찰 모델링)
  velocity *= 0.995;
  
  // 최소 임계값 이하는 0으로 처리
  if (abs(velocity) < VELOCITY_THRESHOLD) {
    velocity = 0.0;
  }
  
  // 최대 속도 추적
  if (abs(velocity) > abs(maxVelocity)) {
    maxVelocity = velocity;
  }
}

void updateState(float distance, bool moving) {
  previousState = currentState;
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
  
  currentState = newState;
  handleStateTransition();
}

void handleStateTransition() {
  // AIR_MOVING 상태에서 속도 누적
  if (currentState == AIR_MOVING) {
    accumulatedVelocity += abs(velocity) * 0.01; // 10ms마다 누적
  }
  
  // 착지 상태로 전환될 때
  if (previousState != GROUNDED && currentState == GROUNDED) {
    // 누적된 속도를 기반으로 반대 속도 계산
    oppositeVelocity = -accumulatedVelocity;
    
    Serial.println("=== 착지 감지! ===");
    Serial.print("현재 속도: "); Serial.print(velocity, 4); Serial.println(" m/s");
    Serial.print("누적 속도: "); Serial.print(accumulatedVelocity, 4); Serial.println(" m/s");
    Serial.print("최대 속도: "); Serial.print(maxVelocity, 4); Serial.println(" m/s");
    Serial.print("필요한 반대 속도: "); Serial.print(oppositeVelocity, 4); Serial.println(" m/s");
    Serial.println("==================");
    
    // 최대 속도 초기화
    maxVelocity = 0.0;
  }
  
  // 공중으로 다시 올라갈 때 속도 초기화
  if (previousState == GROUNDED && currentState != GROUNDED) {
    velocity = 0.0;
    accumulatedVelocity = 0.0;
    Serial.println("공중 상태 전환 - 속도 초기화");
  }
}

String getStateString(MotionState state) {
  switch(state) {
    case AIR_STOP: return "공중/정지";
    case AIR_MOVING: return "공중/이동";
    case GROUNDED: return "착지";
    default: return "알 수 없음";
  }
}

void loop() {
  if (!isCalibrated) return;
  
  unsigned long currentTime = millis();
  if (currentTime - lastTime < 10) return; // 10ms 간격
  
  float deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // 센서 데이터 읽기
  float distance = getDistance();
  
  bool imuAvailable = IMU.accelerationAvailable() && IMU.gyroscopeAvailable();
  if (imuAvailable) {
    IMU.readAcceleration(accelX, accelY, accelZ);
    IMU.readGyroscope(gyroX, gyroY, gyroZ);

    // 가속도 크기 계산 및 필터링
    float accelMag = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ) * 9.81;
    filteredAccelMag = ALPHA * accelMag + (1 - ALPHA) * filteredAccelMag;

    // 속도 계산
    calculateVelocity(deltaTime);
    
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
      Serial.print(" | 현재속도: "); Serial.print(velocity, 4); Serial.print("m/s");
      Serial.print(" | 누적속도: "); Serial.print(accumulatedVelocity, 4); Serial.print("m/s");
      Serial.print(" | 보정가속도: "); Serial.print((accelX - accelBiasX) * 9.81, 3); Serial.print("m/s²");
      Serial.println();
      lastLogTime = currentTime;
    }
  }
}