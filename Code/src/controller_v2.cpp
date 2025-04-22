/*
ControllBoxV2_12102024
MCU STM32F103C8T6
2 NODE 1 CONTROLLER
*/
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

#define Sensor1 Serial3
#define Sensor3 Serial2

#define TimeSirenDelay1 100  // sound duration 
#define TimeSirenDelay2 200 
#define TimeSirenDelay3 500

#define PIN_SW_MODE PB3 
#define PIN_TRIGGER PB8 
#define PIN_SIREN PB9   
#define NUM_ZONES 4     
#define LED_TEST PC13

HardwareSerial Serial_Print(PA10, PA9);  
HardwareSerial Sensor1(USART3);          
HardwareSerial Sensor3(USART2);          

String data_sensor[2];
uint8_t zone[NUM_ZONES] = {0, 0, 0, 0};
bool mode = true;

bool isPlaying = false; 
uint8_t State;
unsigned long pMillis = millis();

unsigned long lastSwitchTime = 0;
unsigned long switchInterval = 10; 


void stopPlaying();
uint8_t findMinZone();
void soundzone1();
void soundzone2();
void soundzone3();
void updateSound();

void process_dataN1(String msg, String header, uint8_t num);
void process_dataN3(String msg, String header, uint8_t num);

void playSound(uint16_t frequency, uint16_t delayTime);
void updateplay();

void setup()
{
  Serial_Print.begin(9600);
  Sensor1.begin(9600); // Sensor1
  Sensor3.begin(9600); // Sensor3

  pinMode(PIN_SW_MODE, INPUT);
  pinMode(PIN_TRIGGER, OUTPUT);
  pinMode(PIN_SIREN, OUTPUT);
  pinMode(LED_TEST, OUTPUT);
  digitalWrite(PIN_TRIGGER, HIGH);
  digitalWrite(PIN_SIREN, LOW);
  mode = true;

  delay(2000);
  Serial_Print.println("Starting...");
  Sensor1.write('P');
  Sensor3.write('P');
}
void loop()
{
  if (Sensor1.available() > 0)
  {
    data_sensor[0] = Sensor1.readStringUntil('\n');
    Serial_Print.println("Data from Sensor1: " + data_sensor[0]);
    process_dataN1(data_sensor[0], "N1", 0);
  }
  if (Sensor3.available() > 0)
  {
    data_sensor[2] = Sensor3.readStringUntil('\n');
    Serial_Print.println("Data from Sensor3: " + data_sensor[2]);
    process_dataN3(data_sensor[2], "N3", 2);
  }
  findMinZone();
  updateSound();
}

void process_dataN1(String msg, String header, uint8_t num)
{
  char c;
  //read data uart
  while (Sensor1.available())
  {
    c = Sensor1.read();
    msg += c;
    delay(2);
  }


  DynamicJsonDocument command(1024);
  deserializeJson(command, msg);

  bool Header_bool = command.containsKey("Header");
  bool distance_bool = command.containsKey("distance");
  bool zone_bool = command.containsKey("zone");

  String Data_header = command["Header"].as<String>();
  int Data_value = command["distance"].as<int>();
  int zone_value = command["zone"].as<int>();
  if (zone_bool)
  {
    zone[num] = zone_value; // Cập nhật zone[num]
    // Serial_Print.println("Zone[" + String(num) + "] = " + String(zone[num]));
  }
  //}
}

void process_dataN3(String msg, String header, uint8_t num)
{
  while (Sensor3.available())
  {
    String msg = "";
    char c;
    while (Sensor3.available())
    {
      c = Sensor3.read();
      msg += c;
      delay(2); 
    }
    // Serial_Print.print("\nReceived JSON: ");
    // Serial_Print.println(msg);

    DynamicJsonDocument command(1024);
    deserializeJson(command, msg);

    bool Header_bool = command.containsKey("Header");
    bool distance_bool = command.containsKey("distance");
    bool zone_bool = command.containsKey("zone");

    String Data_header = command["Header"].as<String>();
    int Data_value = command["distance"].as<int>();
    int zone_value = command["zone"].as<int>();

    if (zone_bool)
    {
      zone[num] = zone_value; // update zone[num]
      Serial_Print.println("Zone[" + String(num) + "] = " + String(zone[num]));
    }
    else
    {
    }
  }
}

uint8_t findMinZone()
{

   uint8_t minZone = 4;
  for (int i = 0; i < NUM_ZONES; i++)
  {
    if (zone[i] != 0 && zone[i] < minZone)
    {
      minZone = zone[i]; 
    }
  }
  
  if (minZone != 1 && minZone != 2 && minZone != 3)
  {
    minZone = 0;
  }
  
  Serial_Print.println("Minimum Zone: " + String(minZone));

  return minZone;
}

void updateSound()
{
  uint8_t minZone = findMinZone(); 
  switch (minZone)
  {
  case 0:
    stopPlaying(); 
    digitalWrite(PIN_TRIGGER, LOW);
    digitalWrite(LED_TEST, LOW);
    // Serial_Print.println("LED_TEST ZONE0 :" + String(minZone));
    break;
  case 1:
    digitalWrite(PIN_TRIGGER, HIGH);
    digitalWrite(LED_TEST, HIGH);
    // Serial_Print.println("LED_TEST ZONE1 :" + String(minZone));
    soundzone1();
    break;
  case 2:
    digitalWrite(PIN_TRIGGER, LOW);
    digitalWrite(LED_TEST, LOW);
    // Serial_Print.println("LED_TEST ZONE2 :" + String(minZone));
    soundzone2();
    break;
  case 3:
    digitalWrite(PIN_TRIGGER, LOW);
    digitalWrite(LED_TEST, LOW);
    // Serial_Print.println("LED_TEST ZONE3 :" + String(minZone));
    soundzone3();
    break;
  }
}

void stopPlaying()
{
  noTone(PIN_SIREN); 
}

void soundzone1()
{
  // Serial_Print.println("XXXXXXXXXXXXXXXXXXXXXX");
    // unsigned long cMillis = millis();
  if (millis() - pMillis >= TimeSirenDelay1) {
    pMillis = millis();
    State = !State;
  }
  if (State) {
    tone(PIN_SIREN, 100);
  } else {
    noTone(PIN_SIREN);
  }
}

void soundzone2()
{
    unsigned long cMillis = millis();

  if (cMillis - pMillis >= TimeSirenDelay2)
  {
    pMillis = millis();
    State = !State;
  }
  if (State)
  {
    tone(PIN_SIREN, 200);
  }
  else
  {
    noTone(PIN_SIREN);
  }
}

void soundzone3()
{
    unsigned long cMillis = millis();

  if (cMillis - pMillis >= TimeSirenDelay3)
  {
    pMillis = millis();
    State = !State;
  }
  if (State)
  {
    tone(PIN_SIREN, 500);
  }
  else
  {
    noTone(PIN_SIREN);
  }
}

