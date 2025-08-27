/*
  스마트 옷장 아두이노 구현
  - 방향 최적화 (1~4 정방향, 5→역3 / 6→역2 / 7→역1, 0/8→무동작)
  - 홀센서: 정방향 D11, 역방향 D12 (방향에 따라 다른 핀 사용)
  - 통신부: AT → CWMODE → CWJAP → CIFSR → CIPMUX → CIPSERVER (응답 시리얼 출력)
  - 앱 패킷: "+IPD:<idx,clothCount>"  예) "+IPD,52:3,2"
  - 예외: n=0이면 무동작
*/

#include <SoftwareSerial.h>

/*──────────── 모터·센서 핀 ────────────*/
#define ENA_PIN      10
#define IN1_PIN       9
#define IN2_PIN       8
#define HALL_FWD_PIN 11   // 정방향용 홀센서
#define HALL_REV_PIN 12   // 역방향용 홀센서

/*──────────── 속도 테이블 ────────────*/
#define SPEED_0  60
#define SPEED_1  60
#define SPEED_2  60
#define SPEED_3  60
#define SPEED_4  60
int currentSpeed = SPEED_0;

/*──────────── 급제동 유지 시간 ────────────*/
#define BRAKE_MS 500   // 급제동 유지 시간(ms)

/*──────────── Wi-Fi(ESP-01 AT) 통신 ────────────*/
SoftwareSerial esp(2, 3);  // UNO: D2(RX)←ESP TX, D3(TX)→ESP RX
const char *SSID = "JM notebook";
const char *PASS = "1234567890a";

/*──────────── AT 유틸 ────────────*/
void at(const String &cmd, uint16_t wait = 3000) {
  esp.println(cmd);
  uint32_t t0 = millis();
  while (millis() - t0 < wait) {
    while (esp.available()) {
      Serial.write(esp.read());
    }
  }
}

/*──────────── “idx(인덱스),cloth(상하의 및 색상 정보)” 수신부 ────────────*/
// +IPD,<id>,<len>:payload  또는 +IPD,<len>:payload 허용 (예: "3,2")
bool readPayload(int &outIdx, int &outCloth) {
  if (!esp.available()) return false;
  String s = esp.readStringUntil('\n');
  s.trim();
  if (!s.startsWith("+IPD")) return false;

  int colon = s.indexOf(':'); if (colon < 0) return false;
  String payload = s.substring(colon + 1); // "3,2"
  payload.trim();

  int comma = payload.indexOf(',');
  if (comma < 0) return false;           // 단일 숫자는 무시
  outIdx   = payload.substring(0, comma).toInt();
  outCloth = payload.substring(comma + 1).toInt();
  return true;
}

/*──────────── 모터 제어부 ────────────*/
void startMotor(int dir) {
  digitalWrite(IN1_PIN, dir > 0);
  digitalWrite(IN2_PIN, dir < 0);
  analogWrite(ENA_PIN, currentSpeed);
}
void stopMotor() {
  analogWrite(ENA_PIN, 0);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
}
// 급제동(현재값 유지)
void brakeMotor() {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, HIGH);
  analogWrite(ENA_PIN, 255);
  delay(BRAKE_MS);
  stopMotor();
}

/*──────────── 옷 개수에 따른 정밀 속도 제어────────────*/
void setSpeedByClothCount(int c) {
  if (c <= 0) { currentSpeed = SPEED_0; return; }
  switch (c) {
    case 1: currentSpeed = SPEED_1; break;
    case 2: currentSpeed = SPEED_2; break;
    case 3: currentSpeed = SPEED_3; break;
    case 4: currentSpeed = SPEED_4; break;
    default: currentSpeed = SPEED_3; break; // 기본 속도
  }
}

/*──────────── 방향 최적화: n=abs(idx) ────────────*/
// n=0/8 → 무동작, 1~4 → 정방향 n스텝, 5→역3 / 6→역2 / 7→역1
void computeDirAndSteps(int idx, int &dir, int &steps) {
  int n = abs(idx);

  if (n == 0) { dir = 0; steps = 0; return; }      // 0 → 무동작
  if (n > 8) n = ((n - 1) % 8) + 1;                // 9→1, 10→2, ...

  if (n <= 3) {                                    // 1,2,3
    dir = +1; steps = n;
  } else if (n == 4) {                             // 4는 그대로 4(정방향)
    dir = +1; steps = 4;
  } else if (n <= 7) {                             // 5,6,7 → 역방향 (8-n)
    dir = -1; steps = 8 - n;                       // 5→3, 6→2, 7→1
  } else {                                         // n==8 → 무동작
    dir = 0; steps = 0;
  }
}

/*──────────── 1/8스텝 이동 (방향별 다른 홀센서 사용) ────────────*/
void moveOptimized(int idx) {
  int dir, steps;
  computeDirAndSteps(idx, dir, steps);

  if (steps == 0) {
    Serial.println("이동 없음\n");
    return;
  }

  // 방향에 따라 센서 핀 선택
  int sensorPin = (dir > 0) ? HALL_FWD_PIN : HALL_REV_PIN;

  Serial.print(dir > 0 ? "정방향 " : "역방향 ");
  Serial.print("1/8스텝×"); Serial.print(steps);
  Serial.print("  speed="); Serial.println(currentSpeed);

  for (int i = 0; i < steps; i++) {
    bool prev = digitalRead(sensorPin);
    startMotor(dir);

    // 홀센서 하강엣지(HIGH→LOW) 1회 = 1 스텝
    while (true) {
      bool now = digitalRead(sensorPin);
      if (prev && !now) {
        Serial.print("  · 스텝 "); Serial.println(i + 1);
        break;
      }
      prev = now;
    }

    // 스텝 종료 동작(현재: 급제동 유지)
    brakeMotor();

    if (i < steps - 1) delay(700); // 관성 억제/휴식
  }
  Serial.println("==> 완료\n");
}

/*──────────── setup / loop ────────────*/
void setup() {
  Serial.begin(9600);
  esp.begin(9600);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  stopMotor();

  // 홀센서 핀 설정(정/역 각각)
  pinMode(HALL_FWD_PIN, INPUT_PULLUP);
  pinMode(HALL_REV_PIN, INPUT_PULLUP);

  // 통신부
  Serial.println("ESP8266 초기화 중…");
  at("AT");
  at("AT+CWMODE=1");
  at("AT+CWJAP=\"" + String(SSID) + "\",\"" + String(PASS) + "\"", 10000);
  at("AT+CIFSR");    // IP 확인
  at("AT+CIPMUX=1");
  at("AT+CIPSERVER=1,8080");
  Serial.println("Wi-Fi 준비 완료! 통신을 시작하자! \n");
}

void loop() {
  int idx, cloth;
  if (!readPayload(idx, cloth)) return;

  Serial.print("idx="); Serial.print(idx);
  Serial.print("  cloth="); Serial.println(cloth);

  setSpeedByClothCount(cloth);
  moveOptimized(idx);
}
