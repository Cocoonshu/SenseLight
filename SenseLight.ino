#define ULTRASONIC_ECHO_PIN              2
#define ULTRASONIC_TRIGGER_PIN           4
#define ULTRASONIC_DISCOVERING_STATE_PIN 13
#define ULTRASONIC_DISCOVERING_INTERVAL  500000
#define LED_0_PIN                        3
#define LED_1_PIN                        5
#define LED_2_PIN                        6
#define LED_3_PIN                        9
#define LED_4_PIN                        10
#define LED_POWER_PIN                    12
#define POWER_BUTTON_PIN                 11
#define POWER_OFF                        0
#define POWER_ON                         1
#define POWER_DISTANCE                   2
#define PWM_ON                           255
#define PWM_OFF                          000
#define PWM_FACTOR                       0.05
#define LED_ALL_ON_DISTANCE              20.0 // cm
#define LED_ALL_OFF_DISTANCE             60.0 // cm
#define LED_COUNT                        5

volatile bool  mReadyToDiscovering          = true;
unsigned long  mEchoStartTime               = 0;
unsigned long  mEchoEndTime                 = 0;
float          mDistance                    = 0.0f;
uint8_t        mPowerMode                   = POWER_DISTANCE;
float          mCurrentPowerLEDValue        = PWM_OFF;
float          mDestPowerLEDValue           = PWM_OFF;
float          mCurrentLEDValues[LED_COUNT] = {PWM_OFF, PWM_OFF, PWM_OFF, PWM_OFF, PWM_OFF};
float          mDestLEDValues[LED_COUNT]    = {PWM_OFF, PWM_OFF, PWM_OFF, PWM_OFF, PWM_OFF};
const uint8_t  LED_PINS[LED_COUNT]          = {LED_0_PIN, LED_1_PIN, LED_2_PIN, LED_3_PIN, LED_4_PIN};

void trigger();
void calcDistance();
void onEcho();
bool isReadyToDiscovering();
void checkPowerButton();
void applyPowerLED();
void applyLightLEDs();
  
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Ultrasonic ranging ready");

  // Led pins setup
  pinMode(LED_POWER_PIN, OUTPUT);
  for (int i = 0; i < LED_COUNT; i++) {
    pinMode(LED_PINS[i], OUTPUT);
  }

  // UltraSonic pins setup
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC_DISCOVERING_STATE_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ULTRASONIC_ECHO_PIN), onEcho, CHANGE);

  // UltraSonic pins initialization
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
  digitalWrite(ULTRASONIC_DISCOVERING_STATE_PIN, LOW);
}

void loop() {
  checkPowerButton();
  applyPowerLED();
  applyLightLEDs();
  if (isReadyToDiscovering()) {
    calcDistance();
    trigger();
  }
}

bool isReadyToDiscovering() {
  if (mReadyToDiscovering) {
    unsigned long now = micros();
    if (now - mEchoEndTime > ULTRASONIC_DISCOVERING_INTERVAL) {
      mReadyToDiscovering = false;
      return true;
    }
  }
  return false;
}

void trigger() {
  digitalWrite(ULTRASONIC_TRIGGER_PIN, HIGH);
  delayMicroseconds(50);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
}

void onEcho() {
  uint8_t echo = digitalRead(ULTRASONIC_ECHO_PIN);
  unsigned long now = micros();
  if (echo == HIGH) {
    mEchoStartTime = now;
    digitalWrite(ULTRASONIC_DISCOVERING_STATE_PIN, HIGH);
  } else {
    mEchoEndTime = now;
    digitalWrite(ULTRASONIC_DISCOVERING_STATE_PIN, LOW);
    mReadyToDiscovering = true;
  }
}

void calcDistance() {
  long deltaTime = mEchoEndTime - mEchoStartTime;
  mDistance = 340L * deltaTime / 2000000.0; // s = vt = 340 m/s * time / 2
  //Serial.print("mDistance = ");Serial.print(mDistance * 100.0);Serial.print("cm, Time = ");Serial.print(deltaTime);Serial.println("us");
}

uint8_t lastButtonValue = LOW;
void checkPowerButton() {
  uint8_t buttonValue = digitalRead(POWER_BUTTON_PIN);
  if (buttonValue != lastButtonValue) {
    lastButtonValue = buttonValue;
    if (buttonValue == HIGH) {
      mPowerMode = (mPowerMode + 1) % 3; // 3 power modes
      switch (mPowerMode) {
        case POWER_OFF: {
          Serial.println("Power: OFF");
        } break;
        
        case POWER_ON: {
          Serial.println("Power: ON");
        } break;
        
        case POWER_DISTANCE: {
          Serial.println("Power: DISTANCE");
        } break;
      }
    }
  }
}

void applyPowerLED() {
  switch (mPowerMode) {
    case POWER_OFF: { mDestPowerLEDValue = PWM_OFF; } break;
    case POWER_ON:  { mDestPowerLEDValue = PWM_ON; }  break;
    case POWER_DISTANCE: {
      if (mDestPowerLEDValue == PWM_OFF) {
        mDestPowerLEDValue = PWM_ON;
      } else {
        mDestPowerLEDValue = PWM_OFF;
      }
    } break;
  }
  mCurrentPowerLEDValue = mCurrentPowerLEDValue + (mDestPowerLEDValue - mCurrentPowerLEDValue) * PWM_FACTOR;
}

void applyLightLEDs() {
  if (mPowerMode == POWER_OFF) {
      for (int i = 0; i < LED_COUNT; i++) {
        mDestLEDValues[i] = PWM_OFF;
        mCurrentLEDValues[i] = mCurrentLEDValues[i] + (mDestLEDValues[i] - mCurrentLEDValues[i]) * PWM_FACTOR;
        analogWrite(LED_PINS[i], mCurrentLEDValues[i]);
      }
  } else if (mPowerMode == POWER_ON) {
      for (int i = 0; i < LED_COUNT; i++) {
        mDestLEDValues[i] = PWM_ON;
        mCurrentLEDValues[i] = mCurrentLEDValues[i] + (mDestLEDValues[i] - mCurrentLEDValues[i]) * PWM_FACTOR;
        analogWrite(LED_PINS[i], mCurrentLEDValues[i]);
      }
  } else {
      float distance     = mDistance * 100.0f;
      int   totalLevel   = 255 * LED_COUNT;
      int   currentLevel = ((distance - LED_ALL_OFF_DISTANCE) / (LED_ALL_ON_DISTANCE - LED_ALL_OFF_DISTANCE)) * totalLevel;
      currentLevel = currentLevel < 0 ? 0 : (currentLevel > totalLevel ? totalLevel : currentLevel);
      Serial.print("level: ");Serial.print(currentLevel);Serial.print("/");Serial.print(totalLevel);
      Serial.print(", distance: ");Serial.print(distance);Serial.println("cm");
      for (int i = 0; i < LED_COUNT; i++) {
        mDestLEDValues[i] = currentLevel > PWM_ON ? PWM_ON: currentLevel;
        currentLevel -= mDestLEDValues[i];
        currentLevel = currentLevel < 0 ? 0 : currentLevel;
        mCurrentLEDValues[i] = mCurrentLEDValues[i] + (mDestLEDValues[i] - mCurrentLEDValues[i]) * PWM_FACTOR;
        analogWrite(LED_PINS[i], mCurrentLEDValues[i]);
      }
  }

  Serial.print("LED: ");
  Serial.print(mCurrentLEDValues[0]);Serial.print(", ");
  Serial.print(mCurrentLEDValues[1]);Serial.print(", ");
  Serial.print(mCurrentLEDValues[2]);Serial.print(", ");
  Serial.print(mCurrentLEDValues[3]);Serial.print(", ");
  Serial.print(mCurrentLEDValues[4]);Serial.println();
}
