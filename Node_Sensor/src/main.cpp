
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include "MyLD2410.h"
#include "ArduinoJson.h"

#define DATA_SIZE 32
// #define SENSOR_UART Serial2
#define RX_PIN 10
#define TX_PIN 9
#define SLAVE4_ADDR 0x24
// #define LD2410_BAUD_RATE 256200

HardwareSerial SENSOR_UART(USART2);
MyLD2410 sensor(SENSOR_UART);

unsigned long nextPrint = 0, printEvery = 100;
String messageToSend = "";
char buffer[DATA_SIZE + 1];
bool control_loop = false;

// unsigned long timegetdata = 0;
unsigned long updategetdata = 100;
unsigned long currentMillis = millis();

void get_command();

void setup()
{
  Serial.begin(9600);
  SENSOR_UART.begin(LD2410_BAUD_RATE);

  if (!sensor.begin())
  {
    //    Serial.println("Không thể giao tiếp với cảm biến.");
    while (true)
    {
    }
  }
  sensor.enhancedMode();
}

void loop()
{

  if ((sensor.check() == MyLD2410::Response::DATA) && (millis() > nextPrint))
  {
    nextPrint = millis() + printEvery;
    int distance = sensor.detectedDistance();
    String zoneId;

    if (distance <= 75)
    {
      zoneId = "1";
    }
    else if (distance <= 150 && distance >= 75)
    {
      zoneId = "2";
    }
    else if (distance <= 235 && distance >= 150)
    {
      zoneId = "3";
    }
    else
    {
      zoneId = "0";
    }
    // messageToSend = "N2," + String(distance) + "," +  zoneId + "|";
    // Serial.print(messageToSend);

    if (control_loop)
    {
      if (millis() - currentMillis >= updategetdata)
      {
        currentMillis = millis();

        String inputString = ""; // char inputString[1024];
        inputString.reserve(1024);
        DynamicJsonDocument sends(1024);
        sends["Header"] = "N2";
        sends["distance"] = distance;
        sends["zone"] = zoneId;
        serializeJson(sends, inputString);
        Serial.println(inputString);
        inputString = "";
      }
    }
  }
  get_command();
}

void get_command()
{
  if (Serial.available() > 0)
  {
    char cmd = Serial.read();
    switch (cmd)
    {
    case 'R':
      Serial.println(messageToSend);
      break;
    case 'S':
      control_loop = false;
      break;
    case 'P':
      control_loop = true;
      break;
    }
  }
}
