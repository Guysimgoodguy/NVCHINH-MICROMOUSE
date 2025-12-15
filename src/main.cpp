#include "config.h"
#include <MPU6050_WE.h>
#include "encoders.h"
#include "systick.h"
#include "motors.h"
#include "profile.h"
#include "sensors.h"
#include "adc_dma.h"
#include "mouse.h"
#include "mpu.h"

Sensors sensors;
Motion motion;
Motors motors;
Profile forwardProfile;
Profile rotationProfile;
Maze maze;
Mouse mouse;
Motor mL(AIN1, AIN2, PWMA, offsetA, STBY);
Motor mR(BIN1, BIN2, PWMB, offsetB, STBY);

void scanI2C()
{
  byte error, address;
  int nDevices;

  BTSerial.println("Scan...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      BTSerial.print("Device found at 0x");
      if (address < 16)
      {
        BTSerial.print("0");
      }
      BTSerial.println(address, HEX);
      nDevices++;
    }
    else if (error == 4)
    {
      BTSerial.print("Unknown error at 0x");
      if (address < 16)
      {
        BTSerial.print("0");
      }
      BTSerial.println(address, HEX);
    }
  }
  if (nDevices == 0)
  {
    BTSerial.println("No I2C\n");
  }
  else
  {
    BTSerial.println("Complete\n");
  }
}

void setup()
{
  BTSerial.begin(9600);
  BTSerial.println("meo");

  Wire.begin();
  Wire.setClock(400000);
  delay(250);

  pinMode(PC13, OUTPUT);
  pinMode(PB12, INPUT);
  pinMode(PB13, INPUT);
  adc_dma_init();
  motors.begin();
  motors.disable_controllers();

  setup_encoders();
  reset_encoders();

  sensors.I2C_INIT();

  setup_systick();

  maze.initialise();
  BTSerial.println(F("Maze cleared"));
}

void blkled(int n)
{
  digitalWrite(PC13, HIGH);
  for (int i = 0; i < n; i++)
  {
    digitalWrite(PC13, LOW);
    delay(100);
    digitalWrite(PC13, HIGH);
    delay(100);
  }
}

int SPD = -100;
bool bmnu = false;
int lmnu = 1;

int btnstu = 0;
int btnstd = 0;
char com[128];
void loop()
{

  btnstu = digitalRead(PB12);
  btnstd = digitalRead(PB13);
  if (btnstd == HIGH)
  {
    while (true)
    {
      sensors.enable();
      sensors.update(1);
      BTSerial.print(sensors.lss.value);
      BTSerial.print(" ");
      BTSerial.print(sensors.ff.value);
      BTSerial.print(" ");
      BTSerial.println(sensors.rss.value);
    }
  }
  if (btnstu == HIGH)
  {

    mouse.run_smooth_turns();
  }
  sensors.enable();
  sensors.update(1);
  if (sensors.lss.value < 70)
  {
    blkled(3);
    delay(2000);
    mouse.search_to(maze.goal());
  }
}
