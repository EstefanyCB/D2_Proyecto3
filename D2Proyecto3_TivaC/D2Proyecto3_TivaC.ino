//******************************************************************************************
//Universidad del Valle de Guatemala
//BE3015: Electronica Digital 2
//Estefany Eleuteria Batz Cantor
//TIVA C - Master
//Proyecto 2.
//******************************************************************************************

//******************************************************************************************
//Librerias
//******************************************************************************************
//Librerias para la tarjeta SD
#include <SPI.h>
#include <SD.h>

//Librerias para la pantalla TFT
#include <stdint.h>
#include <stdbool.h>
#include <TM4C123GH6PM.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#include "bitmaps.h"
#include "font.h"
#include "lcd_registers.h"

//******************************************************************************************
//Pines
//******************************************************************************************
#define BtnSPI PF_0 //Boton derecho
#define BtnSave PF_4 //Boton Izquierdo

#define Rx PD_6
#define Tx PD_7

#define LCD_RST PD_0
#define LCD_CS PD_1
#define LCD_RS PD_2
#define LCD_WR PD_3
#define LCD_RD PE_1

//******************************************************************************************
//Prototipos funcionales
//******************************************************************************************
void MemoriaSD(void);

//Funciones de la pantalla
void LCD_Init(void);
void LCD_CMD(uint8_t cmd);
void LCD_DATA(uint8_t data);
void SetWindows(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);
void LCD_Clear(unsigned int c);
void H_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c);
void V_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c);
void Rect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c);
void FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c);
void LCD_Print(String text, int x, int y, int fontSize, int color, int background);

void LCD_Bitmap(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned char bitmap[]);
void LCD_Sprite(int x, int y, int width, int height, unsigned char bitmap[], int columns, int index, char flip, char offset);

//******************************************************************************************
//Variables globales
//******************************************************************************************
//Variables para el antirebote de los botones
int X=0;
int Y=0;
int EstadoAnterior=0;
int EstadoAnterior2=0;

// Se guarda el valor de la saturación de oxigeno y pulsos por minuto
int SPO2 = 0;
int PPM = 0;

int Datos = 0; // Variable para confirmar que se recibieron ambos datos

//Los datos se convierten a string para imprimirlos en la pantalla TFT
String Porcentaje = "";
String Pulsos = "";

//Tarjeta SD
File archivo;

//Pantalla TFT
int DPINS[] = {PB_0, PB_1, PB_2, PB_3, PB_4, PB_5, PB_6, PB_7};
extern uint8_t fondo[];
extern uint8_t TomandoDato[];
extern uint8_t ECG[];
//***************************************************************************************************************************************
// Inicialización
//***************************************************************************************************************************************
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  //Boton Solicitar dato
  pinMode(BtnSPI, INPUT_PULLUP);

  // Boton guardar Dato
  pinMode(BtnSave, INPUT_PULLUP);
  
  // Pin de SD
  pinMode(PA_3, OUTPUT);
  
  SPI.setModule(0);
    // Estamos Inicializando la tarjeta SD
  if (!SD.begin(PA_3)) {
    Serial.println("initialization failed!");
    return;
  }

  //Pantalla
  SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  Serial.begin(115200);
  GPIOPadConfigSet(GPIO_PORTB_BASE, 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
  LCD_Init();
  LCD_Clear(0x0000);
  LCD_Print("Proyecto 3", 80, 40, 2, 0x0295, 0x0000);
  LCD_Print("Oximetro de pulso", 20, 70, 2, 0x03DF, 0x0000);
  LCD_Print("y", 155, 90, 2, 0x03DF, 0x0000);
  LCD_Print("Frecuencia Cardiaca", 10, 110, 2, 0x03DF, 0x0000);
  LCD_Bitmap(135, 140, 50, 50, CoraInicio);  
}
//***************************************************************************************************************************************
// Loop Infinito
//***************************************************************************************************************************************
void loop() {

//Solicitar valor del peso
  int value = digitalRead(BtnSPI); 
  if((value == LOW)&&(EstadoAnterior==HIGH))
  {
    Serial2.write(1);
    Serial.println("Boton para obtener dato");
    FillRect(0, 0, 319, 239, 0x0000); 
    LCD_Print("Midiendo", 90, 40, 2, 0x03DF, 0x0000);
    LCD_Print("No retire el dedo del sensor", 40, 65, 1, 0x03DF, 0x0000);
    
    for (int x = 0; x < 320 - 32; x++) {
      int anim2 = (x / 24) % 6;
      LCD_Sprite(110, 100, 100, 100, TomandoDato, 6, anim2, 0, 1);
      delay(10); 
    }
    FillRect(0, 0, 319, 239, 0x0000);
   }
   EstadoAnterior=value;

//Se reciben y guardan los dato del sensor
  if(Serial2.available()>0)
  {
    int DatoObtenido = Serial2.read();
    
    if (DatoObtenido == 3)
    {
    Serial.print("PPM: ");
    PPM = Serial2.read();
    Serial.println(PPM);
    
      }
      
     if (DatoObtenido==4)
    {
    Serial.print("SPO2: ");
    SPO2 = Serial2.read();
    Serial.println(SPO2);
    LCD_Clear(0x0000);
    FillRect(0, 0, 319, 239, 0x0000); 
    LCD_Print("PPM", 40, 65, 2, 0xFDC0, 0x0000);
    LCD_Print("%SpO2", 180, 65, 2, 0xFDC0, 0x0000);

    Pulsos =  String(PPM); 
    Porcentaje = String(SPO2);
    
    LCD_Print(Pulsos, 50, 85, 2, 0xFDC0, 0x0000);
    LCD_Print(Porcentaje, 200, 85, 2, 0xFDC0, 0x0000);
    LCD_Bitmap(20, 120, 134, 100, ECG);
    LCD_Bitmap(154, 120, 134, 100, ECG);
    delay(2000);
    FillRect(0, 0, 319, 239, 0x0000);
      }

      if (DatoObtenido == 5){
    FillRect(0, 0, 319, 239, 0x0000); 
    LCD_Print("Vuelve a colocar a intentarlo", 30, 70, 1, 0xFDC0, 0x0000);
    LCD_Print("No se logro realizar", 1, 85, 2, 0xF8E3, 0x0000);
    LCD_Print("la medicion", 60, 105, 2, 0xF8E3, 0x0000);
    LCD_Print("correctamente", 50, 125, 2, 0xF8E3, 0x0000);
    delay(2000);
    FillRect(0, 0, 319, 239, 0x0000);
    }
  }
    
//Guardar el valor de peso
  int value2 = digitalRead(BtnSave);
  if((value2 == LOW)&&(EstadoAnterior2==HIGH))
  {
    Serial2.write(2);
    Serial.println("Boton para guardar dato");
    MemoriaSD();
  }
  EstadoAnterior2=value2;

  if((value==HIGH)&&(value2==HIGH))
  {
  //FillRect(0, 0, 319, 239, 0x0000);
  LCD_Print("Proyecto 3", 80, 40, 2, 0x0295, 0x0000);
  LCD_Print("Oximetro de pulso", 20, 70, 2, 0x03DF, 0x0000);
  LCD_Print("y", 155, 90, 2, 0x03DF, 0x0000);
  LCD_Print("Frecuencia Cardiaca", 10, 110, 2, 0x03DF, 0x0000);
  LCD_Bitmap(135, 140, 50, 50, CoraInicio);
  LCD_Print("B1. Para tomar datos", 70, 200, 1, 0xFDC0, 0x0000);
  LCD_Print("B2. Guardar tomar datos", 60, 210, 1, 0xFDC0, 0x0000);
  }
}

//******************************************************************************************
//Guardar datos en la memoria SD
//******************************************************************************************
void MemoriaSD(void)
{
  //Archivo para guardar los datos
  archivo = SD.open("P3.txt", FILE_WRITE);

  if(archivo)
  {    
    archivo.print("PPM:");
    archivo.print(",");
    archivo.print(PPM);
    archivo.print(",");
    archivo.print("SPO2:");
    archivo.print(",");
    archivo.println(SPO2);
    archivo.close();
    
    Serial.println("Se han registrado exitosamente las PPM y el SPO2 ");
    Serial.print("PPM Guardada: ");
    Serial.print(PPM);
    Serial.print("  SPO2 Guardado: ");
    Serial.println(SPO2);

    FillRect(0, 0, 319, 239, 0x0000); 
    LCD_Print("Se han guardado", 40, 85, 2, 0xFDC0, 0x0000);
    LCD_Print("Exitosamente", 60, 105, 2, 0xFDC0, 0x0000);
    LCD_Print("los datos", 80, 125, 2, 0xFDC0, 0x0000);
    delay(2000);
    FillRect(0, 0, 319, 239, 0x0000);
  }

  else
  {
    Serial.println("No se pudo abrir el archivo");
  }
}

//***************************************************************************************************************************************
// Función para inicializar LCD
//***************************************************************************************************************************************
void LCD_Init(void) {
  pinMode(LCD_RST, OUTPUT);
  pinMode(LCD_CS, OUTPUT);
  pinMode(LCD_RS, OUTPUT);
  pinMode(LCD_WR, OUTPUT);
  pinMode(LCD_RD, OUTPUT);
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(DPINS[i], OUTPUT);
  }
  //****************************************
  // Secuencia de Inicialización
  //****************************************
  digitalWrite(LCD_CS, HIGH);
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_WR, HIGH);
  digitalWrite(LCD_RD, HIGH);
  digitalWrite(LCD_RST, HIGH);
  delay(5);
  digitalWrite(LCD_RST, LOW);
  delay(20);
  digitalWrite(LCD_RST, HIGH);
  delay(150);
  digitalWrite(LCD_CS, LOW);
  //****************************************
  LCD_CMD(0xE9);  // SETPANELRELATED
  LCD_DATA(0x20);
  //****************************************
  LCD_CMD(0x11); // Exit Sleep SLEEP OUT (SLPOUT)
  delay(100);
  //****************************************
  LCD_CMD(0xD1);    // (SETVCOM)
  LCD_DATA(0x00);
  LCD_DATA(0x71);
  LCD_DATA(0x19);
  //****************************************
  LCD_CMD(0xD0);   // (SETPOWER)
  LCD_DATA(0x07);
  LCD_DATA(0x01);
  LCD_DATA(0x08);
  //****************************************
  LCD_CMD(0x36);  // (MEMORYACCESS)
  LCD_DATA(0x40 | 0x80 | 0x20 | 0x08); // LCD_DATA(0x19);
  //****************************************
  LCD_CMD(0x3A); // Set_pixel_format (PIXELFORMAT)
  LCD_DATA(0x05); // color setings, 05h - 16bit pixel, 11h - 3bit pixel
  //****************************************
  LCD_CMD(0xC1);    // (POWERCONTROL2)
  LCD_DATA(0x10);
  LCD_DATA(0x10);
  LCD_DATA(0x02);
  LCD_DATA(0x02);
  //****************************************
  LCD_CMD(0xC0); // Set Default Gamma (POWERCONTROL1)
  LCD_DATA(0x00);
  LCD_DATA(0x35);
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0x02);
  //****************************************
  LCD_CMD(0xC5); // Set Frame Rate (VCOMCONTROL1)
  LCD_DATA(0x04); // 72Hz
  //****************************************
  LCD_CMD(0xD2); // Power Settings  (SETPWRNORMAL)
  LCD_DATA(0x01);
  LCD_DATA(0x44);
  //****************************************
  LCD_CMD(0xC8); //Set Gamma  (GAMMASET)
  LCD_DATA(0x04);
  LCD_DATA(0x67);
  LCD_DATA(0x35);
  LCD_DATA(0x04);
  LCD_DATA(0x08);
  LCD_DATA(0x06);
  LCD_DATA(0x24);
  LCD_DATA(0x01);
  LCD_DATA(0x37);
  LCD_DATA(0x40);
  LCD_DATA(0x03);
  LCD_DATA(0x10);
  LCD_DATA(0x08);
  LCD_DATA(0x80);
  LCD_DATA(0x00);
  //****************************************
  LCD_CMD(0x2A); // Set_column_address 320px (CASET)
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0x3F);
  //****************************************
  LCD_CMD(0x2B); // Set_page_address 480px (PASET)
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0xE0);
  //  LCD_DATA(0x8F);
  LCD_CMD(0x29); //display on
  LCD_CMD(0x2C); //display on

  LCD_CMD(ILI9341_INVOFF); //Invert Off
  delay(120);
  LCD_CMD(ILI9341_SLPOUT);    //Exit Sleep
  delay(120);
  LCD_CMD(ILI9341_DISPON);    //Display on
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para enviar comandos a la LCD - parámetro (comando)
//***************************************************************************************************************************************
void LCD_CMD(uint8_t cmd) {
  digitalWrite(LCD_RS, LOW);
  digitalWrite(LCD_WR, LOW);
  GPIO_PORTB_DATA_R = cmd;
  digitalWrite(LCD_WR, HIGH);
}
//***************************************************************************************************************************************
// Función para enviar datos a la LCD - parámetro (dato)
//***************************************************************************************************************************************
void LCD_DATA(uint8_t data) {
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_WR, LOW);
  GPIO_PORTB_DATA_R = data;
  digitalWrite(LCD_WR, HIGH);
}
//***************************************************************************************************************************************
// Función para definir rango de direcciones de memoria con las cuales se trabajara (se define una ventana)
//***************************************************************************************************************************************
void SetWindows(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) {
  LCD_CMD(0x2a); // Set_column_address 4 parameters
  LCD_DATA(x1 >> 8);
  LCD_DATA(x1);
  LCD_DATA(x2 >> 8);
  LCD_DATA(x2);
  LCD_CMD(0x2b); // Set_page_address 4 parameters
  LCD_DATA(y1 >> 8);
  LCD_DATA(y1);
  LCD_DATA(y2 >> 8);
  LCD_DATA(y2);
  LCD_CMD(0x2c); // Write_memory_start
}
//***************************************************************************************************************************************
// Función para borrar la pantalla - parámetros (color)
//***************************************************************************************************************************************
void LCD_Clear(unsigned int c) {
  unsigned int x, y;
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  SetWindows(0, 0, 319, 239); // 479, 319);
  for (x = 0; x < 320; x++)
    for (y = 0; y < 240; y++) {
      LCD_DATA(c >> 8);
      LCD_DATA(c);
    }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar una línea horizontal - parámetros ( coordenada x, cordenada y, longitud, color)
//***************************************************************************************************************************************
void H_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c) {
  unsigned int i, j;
  LCD_CMD(0x02c); //write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  l = l + x;
  SetWindows(x, y, l, y);
  j = l;// * 2;
  for (i = 0; i < l; i++) {
    LCD_DATA(c >> 8);
    LCD_DATA(c);
  }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar una línea vertical - parámetros ( coordenada x, cordenada y, longitud, color)
//***************************************************************************************************************************************
void V_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c) {
  unsigned int i, j;
  LCD_CMD(0x02c); //write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  l = l + y;
  SetWindows(x, y, x, l);
  j = l; //* 2;
  for (i = 1; i <= j; i++) {
    LCD_DATA(c >> 8);
    LCD_DATA(c);
  }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar un rectángulo - parámetros ( coordenada x, cordenada y, ancho, alto, color)
//***************************************************************************************************************************************
void Rect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c) {
  H_line(x  , y  , w, c);
  H_line(x  , y + h, w, c);
  V_line(x  , y  , h, c);
  V_line(x + w, y  , h, c);
}
//***************************************************************************************************************************************
// Función para dibujar un rectángulo relleno - parámetros ( coordenada x, cordenada y, ancho, alto, color)
//***************************************************************************************************************************************
/*void FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c) {
  unsigned int i;
  for (i = 0; i < h; i++) {
    H_line(x  , y  , w, c);
    H_line(x  , y+i, w, c);
  }
  }
*/

void FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c) {
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);

  unsigned int x2, y2;
  x2 = x + w;
  y2 = y + h;
  SetWindows(x, y, x2 - 1, y2 - 1);
  unsigned int k = w * h * 2 - 1;
  unsigned int i, j;
  for (int i = 0; i < w; i++) {
    for (int j = 0; j < h; j++) {
      LCD_DATA(c >> 8);
      LCD_DATA(c);

      //LCD_DATA(bitmap[k]);
      k = k - 2;
    }
  }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar texto - parámetros ( texto, coordenada x, cordenada y, color, background)
//***************************************************************************************************************************************
void LCD_Print(String text, int x, int y, int fontSize, int color, int background) {
  int fontXSize ;
  int fontYSize ;

  if (fontSize == 1) {
    fontXSize = fontXSizeSmal ;
    fontYSize = fontYSizeSmal ;
  }
  if (fontSize == 2) {
    fontXSize = fontXSizeBig ;
    fontYSize = fontYSizeBig ;
  }

  char charInput ;
  int cLength = text.length();
  //Serial.println(cLength, DEC);
  int charDec ;
  int c ;
  int charHex ;
  char char_array[cLength + 1];
  text.toCharArray(char_array, cLength + 1) ;
  for (int i = 0; i < cLength ; i++) {
    charInput = char_array[i];
    //Serial.println(char_array[i]);
    charDec = int(charInput);
    digitalWrite(LCD_CS, LOW);
    SetWindows(x + (i * fontXSize), y, x + (i * fontXSize) + fontXSize - 1, y + fontYSize );
    long charHex1 ;
    for ( int n = 0 ; n < fontYSize ; n++ ) {
      if (fontSize == 1) {
        charHex1 = pgm_read_word_near(smallFont + ((charDec - 32) * fontYSize) + n);
      }
      if (fontSize == 2) {
        charHex1 = pgm_read_word_near(bigFont + ((charDec - 32) * fontYSize) + n);
      }
      for (int t = 1; t < fontXSize + 1 ; t++) {
        if (( charHex1 & (1 << (fontXSize - t))) > 0 ) {
          c = color ;
        } else {
          c = background ;
        }
        LCD_DATA(c >> 8);
        LCD_DATA(c);
      }
    }
    digitalWrite(LCD_CS, HIGH);
  }
}
//***************************************************************************************************************************************
// Función para dibujar una imagen a partir de un arreglo de colores (Bitmap) Formato (Color 16bit R 5bits G 6bits B 5bits)
//***************************************************************************************************************************************
void LCD_Bitmap(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned char bitmap[]) {
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);

  unsigned int x2, y2;
  x2 = x + width;
  y2 = y + height;
  SetWindows(x, y, x2 - 1, y2 - 1);
  unsigned int k = 0;
  unsigned int i, j;

  for (int i = 0; i < width; i++) {
    for (int j = 0; j < height; j++) {
      LCD_DATA(bitmap[k]);
      LCD_DATA(bitmap[k + 1]);
      //LCD_DATA(bitmap[k]);
      k = k + 2;
    }
  }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar una imagen sprite - los parámetros columns = número de imagenes en el sprite, index = cual desplegar, flip = darle vuelta
//***************************************************************************************************************************************
void LCD_Sprite(int x, int y, int width, int height, unsigned char bitmap[], int columns, int index, char flip, char offset) {
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);

  unsigned int x2, y2;
  x2 =   x + width;
  y2 =    y + height;
  SetWindows(x, y, x2 - 1, y2 - 1);
  int k = 0;
  int ancho = ((width * columns));
  if (flip) {
    for (int j = 0; j < height; j++) {
      k = (j * (ancho) + index * width - 1 - offset) * 2;
      k = k + width * 2;
      for (int i = 0; i < width; i++) {
        LCD_DATA(bitmap[k]);
        LCD_DATA(bitmap[k + 1]);
        k = k - 2;
      }
    }
  } else {
    for (int j = 0; j < height; j++) {
      k = (j * (ancho) + index * width + 1 + offset) * 2;
      for (int i = 0; i < width; i++) {
        LCD_DATA(bitmap[k]);
        LCD_DATA(bitmap[k + 1]);
        k = k + 2;
      }
    }


  }
  digitalWrite(LCD_CS, HIGH);
}
