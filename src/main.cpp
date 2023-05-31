#define BLYNK_PRINT Serial
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <AccelStepper.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

#define LDR1 35
#define LDR2 39
#define LDR3 34
#define LDR4 36

LiquidCrystal_I2C lcd(0x27, 16, 2);
AccelStepper stepperVertical(AccelStepper::FULL2WIRE, 33, 25);
AccelStepper stepperHorizontal(AccelStepper::FULL2WIRE, 26, 27);
Adafruit_INA219 ina219;
BlynkTimer timer;

char auth[] = "DlCq4KuN51ZilTuMK5-hTVpaukxC95Cs";
char ssid[] = "POCO X3 Pro";
char pass[] = "bal12345";

int ldr1, ldr2, ldr3, ldr4; // ldr value
int diffVer, diffHor;
int treshold = 4000;
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;

float Kp = 10;
float Ki = 1;
float Kd = 1;
float error1 = 0, P1 = 0, I1 = 0, D1 = 0, PID_value1 = 0;
float previous_error1 = 0, previous_I1 = 0;
float error2 = 0, P2 = 0, I2 = 0, D2 = 0, PID_value2 = 0;
float previous_error2 = 0, previous_I2 = 0;

void getCurrent(void *parameter);
void pidControl(void *parameter);
void blynkHandler(void *parameter);
void blynkUpdate();
void calculatePID();
void setup()
{
  Serial.begin(115200);
  // Blynk.begin(auth, ssid, pass, "blynk.cloud", 8080);
  // Blynk.begin(auth, ssid, pass);
  // timer.setInterval(2000L, blynkUpdate);

  stepperVertical.setMaxSpeed(500);
  stepperVertical.setAcceleration(400);
  stepperVertical.setSpeed(500);
  stepperHorizontal.setMaxSpeed(500);
  stepperHorizontal.setAcceleration(400);
  stepperHorizontal.setSpeed(500);

  xTaskCreate(getCurrent, "INA219", 1024 * 10, NULL, 0, NULL);
  xTaskCreate(pidControl, "pidTask", 1024 * 10, NULL, 0, NULL);
  xTaskCreate(blynkHandler, "blynkTask", 1024 * 15, NULL, 15, NULL);
}

void loop()
{
  // Blynk.run();
  // timer.run();
  vTaskDelete(NULL);
}
void pidControl(void *pvParameter)
{
  while (1)
  {
    ldr1 = analogRead(LDR1);
    ldr2 = analogRead(LDR2);
    ldr3 = analogRead(LDR3);
    ldr4 = analogRead(LDR4);

    int avgTop = (ldr1 + ldr2) / 2;
    int avgBot = (ldr3 + ldr4) / 2;
    int avgLeft = (ldr1 + ldr3) / 2;
    int avgRight = (ldr2 + ldr4) / 2;

    diffVer = avgTop - avgBot;
    diffHor = avgRight - avgLeft;

    calculatePID();
    if (PID_value1 > -treshold && PID_value1 < treshold)
    {
      stepperHorizontal.move(0);
      // Serial.println("0");
    }
    else
    {
      stepperHorizontal.move(PID_value1);
      // Serial.println(PID_value1);
    }
    if (PID_value2 > -treshold && PID_value2 < treshold)
    {
      stepperVertical.move(0);
      // Serial.println("0");
    }
    else
    {
      stepperVertical.move(PID_value2);
      // Serial.println(PID_value2);
    }
    // Serial.println("Stepper");
    stepperHorizontal.run();
    stepperVertical.run();
    // vTaskDelay(1);
  }
}

void calculatePID()
{ // Motor1
  error1 = diffHor - 250;
  P1 = error1;
  I1 = I1 + previous_I1;
  D1 = error1 - previous_error1;

  PID_value1 = (Kp * P1) + (Ki * I1) + (Kd * D1);

  previous_I1 = I1;
  previous_error1 = error1;

  // Motor2
  error2 = diffVer;
  P2 = error2;
  I2 = I2 + previous_I2;
  D2 = error2 - previous_error2;

  PID_value2 = (Kp * P2) + (Ki * I2) + (Kd * D2);

  previous_I2 = I2;
  previous_error2 = error2;
}

void getCurrent(void *pvParameter)
{
  if (!ina219.begin())
  {
    Serial.println("Failed to find INA219 chip");
    while (1)
    {
      delay(10);
    }
  }
  lcd.init();
  lcd.backlight();
  while (1)
  {
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);

    lcd.setCursor(0, 0);
    lcd.print(busvoltage);
    lcd.print(" v  ");
    lcd.setCursor(8, 0);
    lcd.print(current_mA);
    lcd.print(" mA ");
    lcd.setCursor(0, 1);
    lcd.print(power_mW);
    lcd.print(" mW ");
    vTaskDelay(5000);
    // Serial.print(shuntvoltage);
    // Serial.print(" ");
    // Serial.print(busvoltage);
    // Serial.print(" ");
    // Serial.print(current_mA);
    // Serial.print(" ");
    // Serial.print(power_mW);
    // Serial.print(" ");
    // Serial.println(loadvoltage);
  }
}

void blynkUpdate()
{
  Serial.print("Sent ");
  Serial.print(busvoltage);
  Serial.print(" ");
  Serial.print(current_mA);
  Serial.print(" ");
  Serial.println(power_mW);
  Blynk.virtualWrite(V2, busvoltage);
  Blynk.virtualWrite(V3, current_mA);
  Blynk.virtualWrite(V4, power_mW);
}

void blynkHandler(void *pvParameter)
{
#define BLYNK_PRINT Serial
  Blynk.begin(auth, ssid, pass);
  timer.setInterval(5000L, blynkUpdate);
  while (1)
  {
    Blynk.run();
    timer.run();
    vTaskDelay(3000);
  }
}
