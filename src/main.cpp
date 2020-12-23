#include <Arduino.h>
#include "defines.h"
#include <BLEPeripheral.h>
#include "BLESerial.h"
#include <Adafruit_BMP085.h>
#include <AP3216_WE.h>
#include "MPU6050.h"


BLEService gneralService = BLEService("19b10000e8f2537e4f6cd104768a1214");
BLEUnsignedCharCharacteristic ledCharacteristic = BLEUnsignedCharCharacteristic("0001", BLERead | BLEWrite);
BLEDescriptor ledCharacteristicDescriptor = BLEDescriptor("0001", "LED - byte BIT[B, G, R]");
BLEUnsignedCharCharacteristic keyCharacteristic = BLEUnsignedCharCharacteristic("0009", BLERead);
BLEDescriptor keyCharacteristicDescriptor = BLEDescriptor("0001", "KEY - byte BIT[keyB, keyA]");

BLEUnsignedIntCharacteristic intervalCharacteristic = BLEUnsignedIntCharacteristic("0002", BLERead | BLEWrite);
BLEDescriptor intervalCharacteristicDescriptor = BLEDescriptor("0001", "sensors readout interval (ms)");

BLEService MPU6050Service = BLEService("19b10001e8f2537e4f6cd104768a1214");
BLECharacteristic gyroCharacteristic = BLECharacteristic("0004", BLENotify, 6*sizeof(int16_t));
BLEDescriptor gyroCharacteristicDescriptor = BLEDescriptor("0001", "Accelerometer(A) Gyroscope(G) int16[Gx,Gy,Gz,Ax,Ay,Az]");

BLEService BMP180Service = BLEService("19b10002e8f2537e4f6cd104768a1214");
BLEIntCharacteristic pressureCharacteristic = BLEIntCharacteristic("0005", BLENotify);
BLEDescriptor pressureCharacteristicDescriptor = BLEDescriptor("0001", "Pressure int32 [Pa]");
BLEFloatCharacteristic temperatureCharacteristic = BLEFloatCharacteristic("0006", BLENotify);
BLEDescriptor temperatureCharacteristicDescriptor = BLEDescriptor("0001", "temperature float [degC]");

BLEService AP3216CService = BLEService("19b10003e8f2537e4f6cd104768a1214");
BLEFloatCharacteristic ambientCharacteristic = BLEFloatCharacteristic("0007", BLENotify);
BLEDescriptor ambientCharacteristicDescriptor = BLEDescriptor("0001", "Ambient float [lux]");
BLEUnsignedIntCharacteristic proximityCharacteristic = BLEUnsignedIntCharacteristic("0008", BLENotify);
BLEDescriptor proximityCharacteristicDescriptor = BLEDescriptor("0001", "Proximity unsigned int");

BLESerial BLEConnection(0, 0, 0);

// sensors
TwoWire Wire1(NRF_TWI0, 9, 10);
AP3216_WE ambient_proximity = AP3216_WE();
Adafruit_BMP085 bmp;
MPU6050 accelgyro(0x69);

/*
 *  BLUETOOTH
 */
void startBLE() {
  BLEConnection.setLocalName("UART");

  BLEConnection.setAdvertisedServiceUuid(gneralService.uuid());
  BLEConnection.addAttribute(gneralService);
  BLEConnection.addAttribute(ledCharacteristic);
  BLEConnection.addAttribute(ledCharacteristicDescriptor);
  BLEConnection.addAttribute(keyCharacteristic);
  BLEConnection.addAttribute(keyCharacteristicDescriptor);

  BLEConnection.addAttribute(intervalCharacteristic);
  BLEConnection.addAttribute(intervalCharacteristicDescriptor);
  intervalCharacteristic.setValue(250);

  BLEConnection.setAdvertisedServiceUuid(MPU6050Service.uuid());
  BLEConnection.addAttribute(MPU6050Service);
  BLEConnection.addAttribute(gyroCharacteristic);
  BLEConnection.addAttribute(gyroCharacteristicDescriptor);

  BLEConnection.setAdvertisedServiceUuid(BMP180Service.uuid());
  BLEConnection.addAttribute(BMP180Service);
  BLEConnection.addAttribute(pressureCharacteristic);
  BLEConnection.addAttribute(pressureCharacteristicDescriptor);
  BLEConnection.addAttribute(temperatureCharacteristic);
  BLEConnection.addAttribute(temperatureCharacteristicDescriptor);

  BLEConnection.setAdvertisedServiceUuid(AP3216CService.uuid());
  BLEConnection.addAttribute(AP3216CService);
  BLEConnection.addAttribute(ambientCharacteristic);
  BLEConnection.addAttribute(ambientCharacteristicDescriptor);
  BLEConnection.addAttribute(proximityCharacteristic);
  BLEConnection.addAttribute(proximityCharacteristicDescriptor);

  BLEConnection.begin();
}

void startSensors()
{
  if(!bmp.begin()) {
    BLEConnection.poll();
	  BLEConnection.println("Could not find a valid BMP085 sensor, check wiring!");
  }

	ambient_proximity.init();
 /*
  * Choose between the modes:
  * ALS: ambient light continuous
  * PS: proximity sensor continous
  * ALS_PS: ambient Light and Proximity Sensor continuous (default)
  * ALS_ONCE: ambient light single 
  * PS_ONCE: proximity sensor single 
  * ALS_PS_ONCE: ambient light and proximity sensor single
  * POWER_DOWN: switch off
  * RESET: reset
  */
  ambient_proximity.setMode(ALS_PS_ONCE); // initiates next measurement

  accelgyro.initialize();
  // Cerify the connection:
  BLEConnection.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void setup() {
  pinMode(LED_PIN_R, OUTPUT);
  pinMode(LED_PIN_G, OUTPUT);
  pinMode(LED_PIN_B, OUTPUT);
  pinMode(KEY_B_PIN, INPUT_PULLUP);
  pinMode(KEY_A_PIN, INPUT_PULLUP);
  pinMode(11, INPUT); // AP3216 interrupt pin
  LED_R(LOW);
  LED_G(LOW);
  LED_B(LOW);

  startBLE();

  startSensors();
}

static bool pulse1S = false;
static bool pulse05S = false;
void timeSymbols()
{
    static unsigned long last05Millis = 0;
  static unsigned long last1Millis = 0;
  if (millis() > last05Millis + 500)
  {
    last05Millis = millis();
    pulse05S = true;
    if (millis() > last1Millis + 1000)
    {
      last1Millis = millis();
      pulse1S = true;
    }
  }
  else
  {
    pulse05S = false;
    pulse1S = false;
  }
}

void handleLeds()
{
  if (ledCharacteristic.value())
  {
    unsigned char val = ledCharacteristic.value();
    LED_R(val & 0b001);
    LED_G(val & 0b010);
    LED_B(val & 0b100);
  }
  else
  {
    // turn off leds after 10 seconds
    if ((millis() > 10000) && BLEConnection && !KEY_A)
    {
      LED_R(LOW);
      LED_G(LOW);
      LED_B(LOW);
      return;
    }
    // default behaviour
    LED_B(LOW);
    LED_G(BLEConnection);
    if(pulse05S)
    {
      LED_R(HIGH);
      delay(3);
      LED_R(LOW);
    }
  }
}

void readSensors()
{
  static unsigned long lastRead = 0;
  if(millis() > lastRead + intervalCharacteristic.value())
  {
    lastRead = millis();

    unsigned char keys = KEY_A << 1 | KEY_B;
    keyCharacteristic.setValue(keys);

    if(gyroCharacteristic.subscribed())
    {
      int16_t gyroValues[6];
      accelgyro.getMotion6( &gyroValues[3], &gyroValues[4], &gyroValues[5], 
                            &gyroValues[0], &gyroValues[1], &gyroValues[2]);
      gyroCharacteristic.setValue((unsigned char *)&gyroValues, (unsigned char )sizeof(gyroValues));
      BLEConnection.print("a/g: ");
      BLEConnection.print(gyroValues[0]); BLEConnection.print(" ");
      BLEConnection.print(gyroValues[1]); BLEConnection.print(" ");
      BLEConnection.print(gyroValues[2]); BLEConnection.print(" ");
      BLEConnection.print(gyroValues[3]); BLEConnection.print(" ");
      BLEConnection.print(gyroValues[4]); BLEConnection.print(" ");
      BLEConnection.println(gyroValues[5]);
    }
    if(pressureCharacteristic.subscribed())
    {
      pressureCharacteristic.setValue(bmp.readPressure());
      BLEConnection.print("Pressure= ");
      BLEConnection.print(pressureCharacteristic.value());
      BLEConnection.println(" Pa");
    }
    if(temperatureCharacteristic.subscribed())
    {
      temperatureCharacteristic.setValue(bmp.readTemperature());
      BLEConnection.print("Temperature = ");
      BLEConnection.print(temperatureCharacteristic.value());
      BLEConnection.println(" *C");
    }
  }
  if (pulse1S)
  {
    if(ambientCharacteristic.subscribed() || proximityCharacteristic.subscribed())
    {
      if (ambient_proximity.proximityIsValid())
      {
        if(ambientCharacteristic.subscribed())
        {
          ambientCharacteristic.setValue(ambient_proximity.getAmbientLight());
          BLEConnection.print("Lux= "); BLEConnection.print(ambientCharacteristic.value());
        }
        if(proximityCharacteristic.subscribed())
        {
          proximityCharacteristic.setValue(ambient_proximity.getProximity());
          BLEConnection.print("Prox: "); BLEConnection.println(proximityCharacteristic.value());
        }
        ambient_proximity.setMode(ALS_PS_ONCE); // initiates next measurement
      }
    }
  }
}

void loop() {
  timeSymbols();
  BLEConnection.poll();
  handleLeds();
  
  // loopback
  if (BLEConnection) {
    int byte;
    while ((byte = BLEConnection.read()) > 0) BLEConnection.write(byte);
  }

  readSensors();
  __WFE();
}