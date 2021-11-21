//******************************************************************************************
//Universidad del Valle de Guatemala
//BE3015: Electronica Digital 2
//Estefany Eleuteria Batz Cantor
//Proyecto 3. ESP32 Sensor
//******************************************************************************************

//******************************************************************************************
//Librerias
//******************************************************************************************
#include <Wire.h>

//Sensor
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

//Neopixel
#include <Adafruit_NeoPixel.h>

//******************************************************************************************
//Pines
//******************************************************************************************
#define PIN 13 //Pin
#define NUMPIXELS 1 //No. de neopixels

//Comunicación UART
#define TXD 17
#define RXD 16

#define MAX_BRIGHTNESS 255
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 500
//******************************************************************************************
//Prototipos funcionales
//******************************************************************************************
void Neopixel(void);
void SensorPulsimetro(void);

//******************************************************************************************
//Variables globales
//******************************************************************************************
byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

int Estado = 0;
int X=0;
int PPM = 0;
int SPO2 = 0;
//******************************************************************************************
//Configuraciones
//******************************************************************************************
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200); //Para la comunicación UART con la Tiva

//NeoPixel
  pixels.begin();
  pixels.show();
  
//Sensor
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 69; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  
}

//******************************************************************************************
//Loop Principal
//******************************************************************************************
void loop() {
  if (Serial2.available() > 0)
  {
    int DatoObtenido = Serial2.read();
    Serial.println("");

//Para tomar datos
    if (DatoObtenido == 1)
    {
      Estado = 1;
      }
      
//Para guardar en memoria SD
    if (DatoObtenido ==2)
    {
      Estado = 2;
      }
//En estado 1, se ejecuta el sensor y se envian los datos a la Tiva      
    if(Estado == 1)
    { 
      //Se enciende en Rojo
      pixels.setPixelColor(0, 10, 0, 0);
      pixels.show();

      Serial.println("Estado 1, Obteniendo datos");

      //Se toman los datos
      SensorPulsimetro(); 

      //Se apaga el neopixel
      pixels.setPixelColor(0, 0, 0, 0);
      pixels.show();
    }

//En estado 2, se pone el led Verde cuando la Tiva logra guardar los datos en la SD
    if(Estado == 2)
    { 
      Serial.println("Estado 2, Guardando datos en la SD");
      
      pixels.setPixelColor(0, 0, 10, 0); //Se enciende En verde el neopixel
      pixels.show();
      delay(5000); 
      pixels.setPixelColor(0, 0, 0, 0); //Se apaga el neopixel
      pixels.show();
    }
  }
}

//******************************************************************************************
//Sensor
//******************************************************************************************
void SensorPulsimetro(void) 
{
  Serial.println("Calculando tu frecuencia cardiaca y SPO2");
  Serial.println("No levantes tu dedo hasta que el sensor te lo indique");
  
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
    /*Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);*/
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
      if ((validHeartRate == 1)&&(validSPO2 == 1))
      {
      PPM = PPM+heartRate;
      SPO2 = SPO2+spo2;
      Serial.print(". ");   
      X = 1; //Para salir del While
      }

      else{ 
       Serial.println("Asegurate de haber colocado correctamente tu dedo en el sensor");
       Serial2.write(5);
       X = 0;
        }
    }
    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  
  if (X==1) //Al tomar 25 muestras iguales se sale del while. 
  {   
    PPM = PPM/25; //Se realiza un promedio de los datos obtenidos
    SPO2 = SPO2/25;
    
    Serial.println("");
    Serial.print("Las pulsaciones por minuto son: ");
    Serial.println(PPM);    
    Serial.print("La saturacion de oxigeno es de: ");
    Serial.println(SPO2);
      
    //Se envian los datos obtenidos a la TivaC
    Serial2.write(3);
    Serial2.write(PPM);
    Serial2.write(4);
    Serial2.write(SPO2);
    break;
   }
  }
}

//******************************************************************************************
//Neopixel
//******************************************************************************************
