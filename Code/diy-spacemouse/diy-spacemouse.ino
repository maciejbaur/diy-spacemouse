#include <TinyUSB_Mouse_and_Keyboard.h>
#include <OneButton.h>
#include <Tlv493d.h>
#include <SimpleKalmanFilter.h>

Tlv493d mag = Tlv493d();
SimpleKalmanFilter xFilter(1, 1, 0.2), yFilter(1, 1, 0.2), zFilter(1, 1, 0.2);

// Setup buttons
OneButton button1(27, true);
OneButton button2(24, true);

float xOffset = 0, yOffset = 0, zOffset = 0;
float xCurrent = 0, yCurrent = 0, zCurrent = 0;
unsigned long lastMagUpdate;

int calSamples = 300;     // (300)
int sensivity = 9;        // (8) czulosc
int magRange = 3;         // (3)
int outRange = 127;       // (127)
float xyThreshold = 0.5;  // wartosc powyzej ktorej przesuwa sie kursor
float zThreshold = 1.8;   // wartosc powyzej ktorej jest zmiana z obracania na przesuwanie modelu
// float xCorr = -0.60;      // korekta w pionie
// float yCorr = -0.60;      // korekta w poziomie
// float zCorr =  0.00;      // korekta nacisku

int inRange = magRange * sensivity;
// float lastMagUpdate;

bool isOrbit = false;

void setup() {

  button1.attachClick(goHome);
  button1.attachLongPressStop(goHome);

  button2.attachClick(fitToScreen);
  button2.attachLongPressStop(fitToScreen);

  Mouse.begin();
  Keyboard.begin();

  Serial.begin(9600);
  Wire1.begin();

  // Magnetometer Sensor Init
  mag.begin(Wire1);
  mag.setAccessMode(mag.MASTERCONTROLLEDMODE);
  mag.setCheckFrameCountError(true); // new line from @burke_dev
  mag.disableTemp();
  calibrateSensor();

  // Kalibracja sensora przeniesiona do funkcji calibrateSensor
  // for (int i = 1; i <= calSamples; i++) {
  //   delay(mag.getMeasurementDelay());
  //   mag.updateData();

  //   xOffset += mag.getX();
  //   yOffset += mag.getY();
  //   zOffset += mag.getZ();

  //   Serial.print(".");
  // }

  xOffset = xOffset / calSamples;
  yOffset = yOffset / calSamples;
  zOffset = zOffset / calSamples;

  Serial.println();
  Serial.println(xOffset);
  Serial.println(yOffset);
  Serial.println(zOffset);
}

void loop() {
  button1.tick();
  button2.tick();

  delay(mag.getMeasurementDelay());
  // mag.updateData(); // replaced by new code from @burke_dev

  // Update the magnetometer
  Tlv493d_Error updateDataResult = mag.updateData();
  lastMagUpdate = millis();

  // Value of 2 means ADC hang-up has occurred. Reset the sensor to fix.
  if (updateDataResult == 2) {
    return resetSensor();
  }

  // xCurrent = yFilter.updateEstimate((mag.getY() - xOffset)) + xCorr;     // ruch w pionie z korekta
  // yCurrent = xFilter.updateEstimate((mag.getX() - yOffset)*-1) + yCorr;  // ruch w poziomie z korekta
  // zCurrent = zFilter.updateEstimate((mag.getZ() - zOffset)) + zCorr;     // nacisk z korekta

  xCurrent = yFilter.updateEstimate((mag.getY() - xOffset));     // ruch w pionie
  yCurrent = xFilter.updateEstimate((mag.getX() - yOffset)*-1);  // ruch w poziomie
  zCurrent = zFilter.updateEstimate((mag.getZ() - zOffset));     // nacisk

  if (abs(xCurrent) > xyThreshold || abs(yCurrent) > xyThreshold) {     // jesli wartosci przesuniecia x i y sa powyzej x i y Threshold to ruszaj kursorem
    int xMove = map(xCurrent, -inRange, inRange, -outRange, outRange);
    int yMove = map(yCurrent, -inRange, inRange, -outRange, outRange);

    if (abs(zCurrent) < zThreshold && !isOrbit) {                       // jesli dodatkowo nacisk nie przekracza zThreshold to obracaj model
      Keyboard.press(KEY_LEFT_SHIFT);
      isOrbit = true;
    }

    Mouse.press(MOUSE_MIDDLE);                                          // jesli dodatkowo nacisk przekracza zThreshold to przesuwaj model
    Mouse.move(yMove, xMove, 0);
  } else {
    Mouse.release(MOUSE_MIDDLE);
    if (isOrbit) {
      Keyboard.release(KEY_LEFT_SHIFT);
      isOrbit = false;
    }
  }

  Keyboard.releaseAll();

  Serial.print("Pion: ");
  Serial.print(xCurrent);
  Serial.print(" Poziom: ");
  Serial.print(yCurrent);
  Serial.print(" Nacisk: ");
  Serial.print(zCurrent);
  Serial.println();
}

void goHome() {
  Keyboard.press(KEY_LEFT_CTRL);
  Keyboard.press(KEY_LEFT_SHIFT);
  Keyboard.write('h');

  delay(10);
  Keyboard.releaseAll();
  Serial.println("pressed home");
}

void fitToScreen() {
  Mouse.press(MOUSE_MIDDLE);
  Mouse.release(MOUSE_MIDDLE);
  Mouse.press(MOUSE_MIDDLE);
  Mouse.release(MOUSE_MIDDLE);

  Serial.println("pressed fit");
}

void calibrateSensor() {
  for (int i = 1; i <= calSamples; i++) {
    delay(mag.getMeasurementDelay());
    mag.updateData();

    xOffset += mag.getX();
    yOffset += mag.getY();
    zOffset += mag.getZ();

    Serial.print(".");
  }
}

void resetSensor() {
  mag.end();
  mag.begin(Wire1);
  mag.setAccessMode(mag.MASTERCONTROLLEDMODE);
  mag.setCheckFrameCountError(true);
  mag.disableTemp();
  delay(100);
}