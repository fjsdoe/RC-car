#include <SoftwareSerial.h>
#define RxD 8
#define TxD 9
SoftwareSerial blueToothSerial(RxD, TxD);//블루투스 아두이노 8,9번핀 사용
char readChar;

int motorPin1 = 3;//RC카모터 아두이노 3,5,10,11번핀 사용
int motorPin2 = 5;
int motorPin3 = 10;
int motorPin4 = 11;
int motorSpeed = 130;//RC카모터 속도

int buzzer = 4;//부저 아두이노 4번핀 사용

int ledR = A3;//RGB LED 아두이노 A3,A4,A5번핀 사용
int ledG = A4;
int ledB = A5;

int mq3 = A0;//알콜감지센서 아두이노 A0번핀 사용
int mq3Value = 0;
int mq3Count = 0;

int button = A1;//버튼 아두이노 A1번핀 사용
int buttonValue = 0;

//유량 센서 https://blog.naver.com/mmatelee/221349659341
volatile int flow_frequency;
unsigned char flowsensor = 2;//유량센서 아두이노 2번핀 사용
void flow () {
  flow_frequency++;
}

//지문인식 https://cafe.naver.com/mechawiki/3458
#include <Adafruit_Fingerprint.h>
int getFingerprintIDez();
SoftwareSerial mySerial(6, 7);//지문인식 아두이노 6,7번핀 사용
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

int modeFlag = 0;
int modeCount = 0;
unsigned long lastMillis;



void setup() {
  Serial.begin(9600);

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);

  pinMode(mq3, INPUT);

  pinMode(flowsensor, INPUT);
  digitalWrite(flowsensor, HIGH);
  attachInterrupt(0, flow, RISING);
  sei();

  pinMode(button, INPUT_PULLUP);

  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);

  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);
  led_on(0, 0, 0);

  finger.begin(57600);
  if (finger.verifyPassword()) {
    buzzer_on(2);
  } else {//지문인식센서가 연결안되있으면
    digitalWrite(buzzer, HIGH); delay(5000); digitalWrite(buzzer, LOW);
    Serial.println("not finger sensor");
  }

  rc_stop();
  delay(1000);
  Serial.println("loop start");
}



void loop() {



  if (modeFlag == 0) {//modeFlag값이 0이면 (테스트 대기상태)
    buttonValue = digitalRead(button);//버튼센서값을 buttonValue변수에 저장
    if (buttonValue == 0) {//buttonValue값이 0이면 (버튼이 눌려지면)
      delay(1000);
      Serial.println("test start");
      buzzer_on(2);//부저 2회 울림
      led_on(1, 0, 0);//LED 레드
      modeFlag = 1; //modeFlag값을 1 변경
      modeCount = 100;
    }
  } else if (modeFlag == 4 || modeFlag == 5) {//modeFlag값이 4이거나 5이면 (4=알콜감지상태, 5=알콜미감지상태)
    buttonValue = digitalRead(button);
    if (buttonValue == 0) {//buttonValue값이 0이면 (버튼이 눌려지면)
      rc_stop();//RC카 정지
      delay(1000);
      Serial.println("test reset");
      buzzer_on(2);//부저 2회 울림
      led_on(0, 0, 0);//LED OFF
      modeFlag = 0;//modeFlag값 0 초기화
      finger_set();
    }
  }



  if (millis() - lastMillis > 100) {//millis()값 - lastMillis 값이 100보다 크면 (0.1초 마다 실행되는 부분)
    lastMillis = millis();

    if (modeFlag == 1) {//modeFlag값이 1이면 (버튼을 처음눌러서 테스트가 시작된 상태)
      modeCount--;//modeCount값을 1씩 뺀다

      if (modeCount <= 0) {//modeCount값이 0이거나 0보다 작으면 (일정시간 이상 지문인식이 안되면)
        digitalWrite(buzzer, HIGH); delay(2000); digitalWrite(buzzer, LOW);//부저 2초 울림
        led_on(0, 0, 0);//LED OFF
        modeFlag = 0;//modeFlag값 0 초기화 (테스트 대기상태)
        modeCount = 0;

        Serial.println("test reset");
        return;
      }

      Serial.print("finger test : ");
      Serial.print(modeCount);

      int read_finger = getFingerprintIDez();//지문인식 함수를 실행결과를 read_finger변수에 저장
      Serial.print("     read_finger : ");
      Serial.println(read_finger);
      if (read_finger == 5 || read_finger == 6) {//read_finger값이 5이거나 6이면 (등록된 지문이 인식되면)
        Serial.println("finger success");
        buzzer_on(2);
        led_on(1, 0, 1);//LED 보라색
        modeFlag = 2;//modeFlag값 2 변경
        modeCount = 100;
        flow_frequency = 0;

        blueToothSerial.begin(9600);//블루투스 모듈을 사용하도록 선언
        pinMode(RxD, INPUT);
        pinMode(TxD, OUTPUT);
        blueToothSerial.setTimeout(100);
      } else if (read_finger == -3) {//read_finger값이 -3이면 (등록된 지문이 아니면)
        Serial.println("finger fail");
        digitalWrite(buzzer, HIGH); delay(200); digitalWrite(buzzer, LOW);//부저 0.2초 울림
      }

    } else if (modeFlag == 2) {//modeFlag값이 2이면 (등록된 지문인식 후 유량센서 테스트 상태)
      modeCount--;

      if (modeCount <= 0) {//일정시간 유량센서인식이 안되면
        digitalWrite(buzzer, HIGH); delay(2000); digitalWrite(buzzer, LOW);//부저 2초 울림
        led_on(0, 0, 0);
        modeFlag = 0; finger_set();

        Serial.println("test reset");
        return;
      }

      Serial.print("flow test : ");
      Serial.print(modeCount);

      Serial.print("     flow_frequency : ");
      Serial.println(flow_frequency);//유량센서값 출력

      if (flow_frequency > 10) {//flow_frequency값이 10보다크면 (유량센서감지 = 입김을 불었음)
        buzzer_on(2);
        led_on(0, 0, 1);//LED 파란색
        modeFlag = 3; //modeFlag값 3 변경
        modeCount = 100;
      }
      flow_frequency = 0;

    } else if (modeFlag == 3) {//modeFlag값이 3이면 (입김감지 후 알콜센서 테스트 상태)
      modeCount--;

      if (modeCount <= 0) {
        digitalWrite(buzzer, HIGH); delay(2000); digitalWrite(buzzer, LOW);
        led_on(0, 0, 0);
        modeFlag = 0; finger_set();

        Serial.println("test reset");
        return;
      }

      mq3Value = analogRead(mq3);//mq3Value변수에 알콜센서값 저장
      Serial.print("mq3 test : ");
      Serial.print(modeCount);

      Serial.print("     mq3 : ");
      Serial.println(mq3Value);

      if (mq3Value > 550) {//mq3Value값이 500보다 크면
        digitalWrite(buzzer, HIGH);//부저 ON
        led_on(1, 0, 0);//레드 빨간색
        modeFlag = 4;//modeFlag값 4 변경 (알콜감지상태)
        mq3Count = 0;
        Serial.println("mq3 fail");
      } else if (mq3Value <= 550) {//mq3Value값이 500보다 크지 않으면
        mq3Count++;
        if (mq3Count > 50) {//mq3Count값이 50보다 크면 (일정시간 알콜감지가 안되면)
          mq3Count = 0;
          Serial.println("mq3 success");
          buzzer_on(3);//부저 3회 울림
          led_on(0, 1, 0);//LED 초록색
          modeFlag = 5;//modeFlag값 5 변경 (알콜미감지상태)
        }
      } else {
        mq3Count = 0;
      }
    }
  }



  if (modeFlag == 5) {//알콜미감지상태

    mq3Value = analogRead(mq3);//mq3Value변수에 알콜센서값 저장
    Serial.print("mq3 : ");
    Serial.println(mq3Value);

    if (mq3Value > 550) {//mq3Value값이 550보다 크면
      digitalWrite(buzzer, HIGH);//부저 ON
      rc_stop();//RC카 정지
      led_on(1, 0, 0);//레드 빨간색
      modeFlag = 4;//modeFlag값 4 변경 (알콜감지상태)
      mq3Count = 0;
      Serial.println("mq3 fail");
    }

    if (blueToothSerial.available()) {//블루투스 데이터가 수신되면
      readChar = blueToothSerial.read();//readChar 변수에 수신데이터 저장
      Serial.println(readChar);
      if (modeFlag == 5) {//modeFlag값이 5이면 (지문, 알콜 테스트후 통과한 상태)
        if (readChar == 's') {//블루투스 수신 데이터가 s이면
          rc_stop();//RC카 정지 함수
        } else if (readChar == 'r') {
          rc_right();//RC카 우회전함수
        } else if (readChar == 'l') {
          rc_left();//RC카 좌회전 함수
        } else if (readChar == 'b') {
          rc_back();//RC카 후진 함수
        } else if (readChar == 'g') {
          rc_go();//RC카 전진 함수
        }
      } else {
        rc_stop();//RC카 정지 함수
      }
    }
  }



  delay(5);//0.005초 휴식
}



void led_on(int r, int g, int b) {//LED 제어 함수
  digitalWrite(ledR, r);
  digitalWrite(ledG, g);
  digitalWrite(ledB, b);
}
void buzzer_on(int n) {//n번만큼 부저를 켜는 함수
  for (int i = 0; i < n; i++) {
    digitalWrite(buzzer, HIGH); delay(150);
    digitalWrite(buzzer, LOW); delay(150);
  }
}
void rc_go() {//RC카 전진 함수
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, motorSpeed);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, motorSpeed);
}
void rc_back() {//RC카 후진 함수
  analogWrite(motorPin1, motorSpeed);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, motorSpeed);
  analogWrite(motorPin4, 0);
}
void rc_right() {//RC카 우회전 함수
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, motorSpeed);
  analogWrite(motorPin3, motorSpeed);
  analogWrite(motorPin4, 0);
}
void rc_left() {//RC카 좌회전 함수
  analogWrite(motorPin1, motorSpeed);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, motorSpeed);
}
void rc_stop() {//RC카 정지 함수
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, 0);
}



//지문인식 오픈소스
void finger_set() {
  finger.begin(57600);
  if (finger.verifyPassword()) {
    buzzer_on(2);
  } else {
    digitalWrite(buzzer, HIGH); delay(5000); digitalWrite(buzzer, LOW);
    Serial.println("not finger sensor");
  }
}
uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.println("No finger detected");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }
  // OK success!
  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }
  // OK converted!
  p = finger.fingerFastSearch();
  if (p == FINGERPRINT_OK) {
    Serial.println("Found a print match!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_NOTFOUND) {
    Serial.println("Did not find a match");
    return p;
  } else {
    Serial.println("Unknown error");
    return p;
  }
  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);
}
// returns -1 if failed, otherwise returns ID #
int getFingerprintIDez() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)  return -2;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)  return -3;

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);
  return finger.fingerID;
}
