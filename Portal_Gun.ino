#include <TM74HC595Display.h>
#include <RotaryEncoder.h>
#define ROTARYSTEPS 1
#define ROTARYMIN 0
#define ROTARYMAX 999
#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex
// Setup a RoraryEncoder for pins A2 and A3:
RotaryEncoder encoder(22, 23);
RTC_DATA_ATTR int lastPos = 137;
unsigned long previousMillis = 0;
const long interval = 30000; // interval in ms for sending temperature data to thingspeak
int SCLK = 14;
int RCLK = 12;
int DIO = 13;
int un = 7;
int deux = 3;
int trois = 1;
int pinv1 = 25;
int pinv2 = 26;
int pinv3 = 27;
boolean ledStatev1 = false;
boolean ledStatev2 = false;
boolean ledStatev3 = false;
int pinb1 = 18;
int pinb2 = 19;
int pinb3 = 21;
boolean ledStateb1 = false;
boolean ledStateb2 = false;
boolean ledStateb3 = false;

const int  buttonPin = 33;    // the pin that the pushbutton is attached to
int actif = 1;
int  dispState = 0;
int  dispStateCnt = 0;
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button
TM74HC595Display disp(SCLK, RCLK, DIO);
unsigned char LED_0F[29];

void loop1(void *pvParameters) {
  while (1) {
    vTaskDelay( 3 ); // wait / yield time to other tasks
    if (actif == 1) {
      if (dispState == 1) {
        disp.send(LED_0F[un], 0b0001);
        vTaskDelay( 3 ); // wait / yield time to other tasks
        disp.send(LED_0F[deux], 0b0010);
        vTaskDelay( 3 ); // wait / yield time to other tasks
        disp.send(LED_0F[trois], 0b0100);
        vTaskDelay( 3 ); // wait / yield time to other tasks
        disp.send(LED_0F[12], 0b1000);    //send digital "C" to 4th indicator
      } else {
        disp.send(LED_0F[-2], 0b0000);
        vTaskDelay( 100 ); // wait / yield time to other tasks
      }
    } else {
      vTaskDelay( 100 ); // wait / yield time to other tasks
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT);
  pinMode(pinv1, OUTPUT);
  pinMode(pinv2, OUTPUT);
  pinMode(pinv3, OUTPUT);
  pinMode(pinb1, OUTPUT);
  pinMode(pinb2, OUTPUT);
  pinMode(pinb3, OUTPUT);
  digitalWrite(pinv1, LOW);
  digitalWrite(pinv2, LOW);
  digitalWrite(pinv3, LOW);
  digitalWrite(pinb1, LOW);
  digitalWrite(pinb2, LOW);
  digitalWrite(pinb3, LOW);
  LED_0F[0] = 0xC0; //0
  LED_0F[1] = 0xF9; //1
  LED_0F[2] = 0xA4; //2
  LED_0F[3] = 0xB0; //3
  LED_0F[4] = 0x99; //4
  LED_0F[5] = 0x92; //5
  LED_0F[6] = 0x82; //6
  LED_0F[7] = 0xF8; //7
  LED_0F[8] = 0x80; //8
  LED_0F[9] = 0x90; //9
  LED_0F[10] = 0x88; //A
  LED_0F[11] = 0x83; //b
  LED_0F[12] = 0xC6; //C
  LED_0F[13] = 0xA1; //d
  LED_0F[14] = 0x86; //E
  LED_0F[15] = 0x8E; //F
  LED_0F[16] = 0xC2; //G
  LED_0F[17] = 0x89; //H
  LED_0F[18] = 0xF9; //I
  LED_0F[19] = 0xF1; //J
  LED_0F[20] = 0xC3; //L
  LED_0F[21] = 0xA9; //n
  LED_0F[22] = 0xC0; //O
  LED_0F[23] = 0x8C; //P
  LED_0F[24] = 0x98; //q
  LED_0F[25] = 0x92; //S
  LED_0F[26] = 0xC1; //U
  LED_0F[27] = 0x91; //Y
  LED_0F[28] = 0xFE; //hight -
  encoder.setPosition(lastPos);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0); //1 = High, 0 = Low
  xTaskCreatePinnedToCore(loop1, "loop1", 8192, NULL, 2, NULL, 0);
  dispState = 1;
  dispStateCnt = 5000;
}

void portal() {
  actif = 0;
  Serial.println("Opening portal");
  ledStatev1 = random(2);
  ledStatev2 = random(2);
  ledStatev3 = random(2);
  digitalWrite(pinv1, ledStatev1);
  digitalWrite(pinv2, ledStatev2);
  digitalWrite(pinv3, ledStatev3);
  ledStateb1 = random(2);
  ledStateb2 = random(2);
  ledStateb3 = random(2);
  digitalWrite(pinb1, ledStateb1);
  digitalWrite(pinb2, ledStateb2);
  digitalWrite(pinb3, ledStateb3);
  disp.send(0xf3, 0b1111);
  delay(20);
  ledStatev1 = random(2);
  ledStatev2 = random(2);
  ledStatev3 = random(2);
  digitalWrite(pinv1, ledStatev1);
  digitalWrite(pinv2, ledStatev2);
  digitalWrite(pinv3, ledStatev3);
  ledStateb1 = random(2);
  ledStateb2 = random(2);
  ledStateb3 = random(2);
  digitalWrite(pinb1, ledStateb1);
  digitalWrite(pinb2, ledStateb2);
  digitalWrite(pinb3, ledStateb3);
  disp.send(0xf4, 0b1111);
  delay(20);
  ledStatev1 = random(2);
  ledStatev2 = random(2);
  ledStatev3 = random(2);
  digitalWrite(pinv1, ledStatev1);
  digitalWrite(pinv2, ledStatev2);
  digitalWrite(pinv3, ledStatev3);
  ledStateb1 = random(2);
  ledStateb2 = random(2);
  ledStateb3 = random(2);
  digitalWrite(pinb1, ledStateb1);
  digitalWrite(pinb2, ledStateb2);
  digitalWrite(pinb3, ledStateb3);
  disp.send(0xf5, 0b1111);
  delay(20);
  ledStatev1 = random(2);
  ledStatev2 = random(2);
  ledStatev3 = random(2);
  digitalWrite(pinv1, ledStatev1);
  digitalWrite(pinv2, ledStatev2);
  digitalWrite(pinv3, ledStatev3);
  ledStateb1 = random(2);
  ledStateb2 = random(2);
  ledStateb3 = random(2);
  digitalWrite(pinb1, ledStateb1);
  digitalWrite(pinb2, ledStateb2);
  digitalWrite(pinb3, ledStateb3);
  disp.send(0xf6, 0b1111);
  delay(20);
  ledStatev1 = random(2);
  ledStatev2 = random(2);
  ledStatev3 = random(2);
  digitalWrite(pinv1, ledStatev1);
  digitalWrite(pinv2, ledStatev2);
  digitalWrite(pinv3, ledStatev3);
  ledStateb1 = random(2);
  ledStateb2 = random(2);
  ledStateb3 = random(2);
  digitalWrite(pinb1, ledStateb1);
  digitalWrite(pinb2, ledStateb2);
  digitalWrite(pinb3, ledStateb3);
  disp.send(0xf7, 0b1111);
  ledStatev1 = random(2);
  ledStatev2 = random(2);
  ledStatev3 = random(2);
  digitalWrite(pinv1, ledStatev1);
  digitalWrite(pinv2, ledStatev2);
  digitalWrite(pinv3, ledStatev3);
  ledStateb1 = random(2);
  ledStateb2 = random(2);
  ledStateb3 = random(2);
  digitalWrite(pinb1, ledStateb1);
  digitalWrite(pinb2, ledStateb2);
  digitalWrite(pinb3, ledStateb3);
  disp.send(0xf3, 0b1111);
  delay(20);
  ledStatev1 = random(2);
  ledStatev2 = random(2);
  ledStatev3 = random(2);
  digitalWrite(pinv1, ledStatev1);
  digitalWrite(pinv2, ledStatev2);
  digitalWrite(pinv3, ledStatev3);
  ledStateb1 = random(2);
  ledStateb2 = random(2);
  ledStateb3 = random(2);
  digitalWrite(pinb1, ledStateb1);
  digitalWrite(pinb2, ledStateb2);
  digitalWrite(pinb3, ledStateb3);
  disp.send(0xf4, 0b1111);
  delay(20);
  ledStatev1 = random(2);
  ledStatev2 = random(2);
  ledStatev3 = random(2);
  digitalWrite(pinv1, ledStatev1);
  digitalWrite(pinv2, ledStatev2);
  digitalWrite(pinv3, ledStatev3);
  ledStateb1 = random(2);
  ledStateb2 = random(2);
  ledStateb3 = random(2);
  digitalWrite(pinb1, ledStateb1);
  digitalWrite(pinb2, ledStateb2);
  digitalWrite(pinb3, ledStateb3);
  disp.send(0xf5, 0b1111);
  delay(20);
  ledStatev1 = random(2);
  ledStatev2 = random(2);
  ledStatev3 = random(2);
  digitalWrite(pinv1, ledStatev1);
  digitalWrite(pinv2, ledStatev2);
  digitalWrite(pinv3, ledStatev3);
  ledStateb1 = random(2);
  ledStateb2 = random(2);
  ledStateb3 = random(2);
  digitalWrite(pinb1, ledStateb1);
  digitalWrite(pinb2, ledStateb2);
  digitalWrite(pinb3, ledStateb3);
  disp.send(0xf6, 0b1111);
  delay(20);
  ledStatev1 = random(2);
  ledStatev2 = random(2);
  ledStatev3 = random(2);
  digitalWrite(pinv1, ledStatev1);
  digitalWrite(pinv2, ledStatev2);
  digitalWrite(pinv3, ledStatev3);
  ledStateb1 = random(2);
  ledStateb2 = random(2);
  ledStateb3 = random(2);
  digitalWrite(pinb1, ledStateb1);
  digitalWrite(pinb2, ledStateb2);
  digitalWrite(pinb3, ledStateb3);
  disp.send(0xf7, 0b1111);
  delay(50);
  digitalWrite(pinv1, HIGH);
  digitalWrite(pinv2, HIGH);
  digitalWrite(pinv3, HIGH);
  digitalWrite(pinb1, HIGH);
  digitalWrite(pinb2, HIGH);
  digitalWrite(pinb3, HIGH);
  disp.send(LED_0F[5], 0b1111);
  delay(700);
  disp.send(LED_0F[4], 0b1111);
  delay(700);
  disp.send(LED_0F[3], 0b1111);
  delay(700);
  disp.send(LED_0F[2], 0b1111);
  delay(700);
  disp.send(LED_0F[1], 0b1111);
  delay(700);
  disp.send(LED_0F[0], 0b1111);
  delay(700);
  disp.send(0xf3, 0b1111);
  ledStatev1 = random(2);
  ledStatev2 = random(2);
  ledStatev3 = random(2);
  digitalWrite(pinv1, ledStatev1);
  digitalWrite(pinv2, ledStatev2);
  digitalWrite(pinv3, ledStatev3);
  ledStateb1 = random(2);
  ledStateb2 = random(2);
  ledStateb3 = random(2);
  digitalWrite(pinb1, ledStateb1);
  digitalWrite(pinb2, ledStateb2);
  digitalWrite(pinb3, ledStateb3);
  delay(20);
  disp.send(0xf4, 0b1111);
  ledStatev1 = random(2);
  ledStatev2 = random(2);
  ledStatev3 = random(2);
  digitalWrite(pinv1, ledStatev1);
  digitalWrite(pinv2, ledStatev2);
  digitalWrite(pinv3, ledStatev3);
  ledStateb1 = random(2);
  ledStateb2 = random(2);
  ledStateb3 = random(2);
  digitalWrite(pinb1, ledStateb1);
  digitalWrite(pinb2, ledStateb2);
  digitalWrite(pinb3, ledStateb3);
  delay(20);
  disp.send(0xf5, 0b1111);
  ledStatev1 = random(2);
  ledStatev2 = random(2);
  ledStatev3 = random(2);
  digitalWrite(pinv1, ledStatev1);
  digitalWrite(pinv2, ledStatev2);
  digitalWrite(pinv3, ledStatev3);
  ledStateb1 = random(2);
  ledStateb2 = random(2);
  ledStateb3 = random(2);
  digitalWrite(pinb1, ledStateb1);
  digitalWrite(pinb2, ledStateb2);
  digitalWrite(pinb3, ledStateb3);
  delay(20);
  disp.send(0xf6, 0b1111);
  ledStatev1 = random(2);
  ledStatev2 = random(2);
  ledStatev3 = random(2);
  digitalWrite(pinv1, ledStatev1);
  digitalWrite(pinv2, ledStatev2);
  digitalWrite(pinv3, ledStatev3);
  ledStateb1 = random(2);
  ledStateb2 = random(2);
  ledStateb3 = random(2);
  digitalWrite(pinb1, ledStateb1);
  digitalWrite(pinb2, ledStateb2);
  digitalWrite(pinb3, ledStateb3);
  delay(20);
  disp.send(0xf7, 0b1111);
  ledStatev1 = random(2);
  ledStatev2 = random(2);
  ledStatev3 = random(2);
  digitalWrite(pinv1, ledStatev1);
  digitalWrite(pinv2, ledStatev2);
  digitalWrite(pinv3, ledStatev3);
  ledStateb1 = random(2);
  ledStateb2 = random(2);
  ledStateb3 = random(2);
  digitalWrite(pinb1, ledStateb1);
  digitalWrite(pinb2, ledStateb2);
  digitalWrite(pinb3, ledStateb3);
  delay(50);
  disp.send(0xf3, 0b1111);
  ledStatev1 = random(2);
  ledStatev2 = random(2);
  ledStatev3 = random(2);
  digitalWrite(pinv1, ledStatev1);
  digitalWrite(pinv2, ledStatev2);
  digitalWrite(pinv3, ledStatev3);
  ledStateb1 = random(2);
  ledStateb2 = random(2);
  ledStateb3 = random(2);
  digitalWrite(pinb1, ledStateb1);
  digitalWrite(pinb2, ledStateb2);
  digitalWrite(pinb3, ledStateb3);
  delay(20);
  disp.send(0xf4, 0b1111);
  ledStatev1 = random(2);
  ledStatev2 = random(2);
  ledStatev3 = random(2);
  digitalWrite(pinv1, ledStatev1);
  digitalWrite(pinv2, ledStatev2);
  digitalWrite(pinv3, ledStatev3);
  ledStateb1 = random(2);
  ledStateb2 = random(2);
  ledStateb3 = random(2);
  digitalWrite(pinb1, ledStateb1);
  digitalWrite(pinb2, ledStateb2);
  digitalWrite(pinb3, ledStateb3);
  delay(20);
  disp.send(0xf5, 0b1111);
  ledStatev1 = random(2);
  ledStatev2 = random(2);
  ledStatev3 = random(2);
  digitalWrite(pinv1, ledStatev1);
  digitalWrite(pinv2, ledStatev2);
  digitalWrite(pinv3, ledStatev3);
  ledStateb1 = random(2);
  ledStateb2 = random(2);
  ledStateb3 = random(2);
  digitalWrite(pinb1, ledStateb1);
  digitalWrite(pinb2, ledStateb2);
  digitalWrite(pinb3, ledStateb3);
  delay(20);
  disp.send(0xf6, 0b1111);
  ledStatev1 = random(2);
  ledStatev2 = random(2);
  ledStatev3 = random(2);
  digitalWrite(pinv1, ledStatev1);
  digitalWrite(pinv2, ledStatev2);
  digitalWrite(pinv3, ledStatev3);
  ledStateb1 = random(2);
  ledStateb2 = random(2);
  ledStateb3 = random(2);
  digitalWrite(pinb1, ledStateb1);
  digitalWrite(pinb2, ledStateb2);
  digitalWrite(pinb3, ledStateb3);
  delay(20);
  disp.send(0xf7, 0b1111);
  ledStatev1 = random(2);
  ledStatev2 = random(2);
  ledStatev3 = random(2);
  digitalWrite(pinv1, ledStatev1);
  digitalWrite(pinv2, ledStatev2);
  digitalWrite(pinv3, ledStatev3);
  ledStateb1 = random(2);
  ledStateb2 = random(2);
  ledStateb3 = random(2);
  digitalWrite(pinb1, ledStateb1);
  digitalWrite(pinb2, ledStateb2);
  digitalWrite(pinb3, ledStateb3);
  delay(50);
  digitalWrite(pinv1, LOW);
  digitalWrite(pinv2, LOW);
  digitalWrite(pinv3, LOW);
  digitalWrite(pinb1, LOW);
  digitalWrite(pinb2, LOW);
  digitalWrite(pinb3, LOW);
  actif = 1;
}

void loop() {
  trois = lastPos / 100;
  deux = lastPos % 100 / 10;
  un = lastPos % 10;
  buttonState = digitalRead(buttonPin);
  if (buttonState != lastButtonState) {
    if (buttonState == LOW) {
      // if the current state is LOW then the button has been pushed
      if (dispState == 1) {
        portal();
      }
      dispState = 1;
      dispStateCnt = 5000;
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;
  if (dispState == 1) {
    if (dispStateCnt > 0) {
      dispStateCnt--;
      delay(1);
    } else {
      dispState = 0;
      unsigned long currentMillis = millis();
      previousMillis = currentMillis;
      delay(50);
    }
  } else {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      Serial.println("Bonne nuit");
      esp_deep_sleep_start();
    }
  }
  encoder.tick();
  // get the current physical position and calc the logical position
  int newPos = encoder.getPosition() * ROTARYSTEPS;
  if (newPos < ROTARYMIN) {
    encoder.setPosition(ROTARYMIN / ROTARYSTEPS);
    newPos = ROTARYMIN;
  } else if (newPos > ROTARYMAX) {
    encoder.setPosition(ROTARYMAX / ROTARYSTEPS);
    newPos = ROTARYMAX;
  } // if
  if (lastPos != newPos) {
    dispState = 1;
    dispStateCnt = 5000;
    Serial.print(newPos);
    Serial.println();
    lastPos = newPos;
  } // if
}
