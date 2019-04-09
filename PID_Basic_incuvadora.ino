

/********************************************************
   PID Basic contro, de temperatura y humedad de una
   incubadora
 ********************************************************/
#include <math.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Time.h>
#include "DHT.h"
#define DHTPIN 8
#define DHTTYPE DHT22
#define RELAY 5
#define FAN 6
#define SERVOa1      0
#define SERVOa2      90
#define tROTARHUEVOS 10800000//(rotar huevos cada 2 horas(7200000 mils))
#define pSERVO       A3
#define PIN_OUTPUT 9
float TEMP = 0;
float HUM;

boolean statusServo = false;
unsigned long time1, previoTime = 0;
int periodo = 5000;
unsigned long TiempoAhora = 0;

// configurando el lcd display:
const int rs = 2, en = 3, d4 = 4, d5 = 9, d6 = 10, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
// put your setup code here, to run once:


//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp = 10, Ki = 0.4, Kd = 0.6;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
// iniciando el sensor de temperatura y humedad
DHT dht(DHTPIN, DHTTYPE);

Servo servo; // inicia el servo

// DEFINICION DE CARACTERES PERSONALIZADOS
byte humedad[8] = {
  0b00000100,
  0b00000100,
  0b00001110,
  0b00001110,
  0b00011111,
  0b00011111,
  0b00011111,
  0b00001110,
};

byte Temperatura[8] = {
  0b00001110,
  0b00001010,
  0b00001010,
  0b00001010,
  0b00001010,
  0b00010001,
  0b00010001,
  0b00001110,
};

byte grados[8] = {
  0b00000000,
  0b00001110,
  0b00001010,
  0b00001110,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
};

byte arrowr[8] = {
  0b00001000,
  0b00000100,
  0b00000010,
  0b00011111,
  0b00000010,
  0b00000100,
  0b00001000,
  0b00000000,
};

byte arrowu[8] = {
  0b00000100,
  0b00001110,
  0b00010101,
  0b00000100,
  0b00000100,
  0b00000100,
  0b00000100,
  0b00000000,
};

byte arrowd[8] = {
  0b00000100,
  0b00000100,
  0b00000100,
  0b00000100,
  0b00010101,
  0b00001110,
  0b00000100,
  0b00000000,
};

byte arrowl[8] = {
  0b00000010,
  0b00000100,
  0b00001000,
  0b00011111,
  0b00001000,
  0b00000100,
  0b00000010,
  0b00000000,
};

void setup() {
  //Serial.begin(9600);
  servo.attach(A3,600,2000); // servo en el pin A3
  servo.write(SERVOa1);
  Wire.begin();
  dht.begin();
  lcd.begin(16, 2);

  // ENVIAR LOS MAPAS DE BITS AL CONTROLADOR DE PANTALLA
  lcd.createChar (0, humedad);
  lcd.createChar (1, Temperatura);
  lcd.createChar (2, grados);
  lcd.createChar (3, arrowr);
  lcd.createChar (4, arrowu);
  lcd.createChar (5, arrowd);
  lcd.createChar (6, arrowl);


  pinMode(RELAY, OUTPUT);
  pinMode(FAN, OUTPUT);
  //initialize the variables we're linked to
  //  Input = analogRead(PIN_INPUT);
  Setpoint = 37,7;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  

}

void loop()
{

  int HUM = dht.readHumidity();
  float TEMP = dht.readTemperature();
  // Proteccion del sistema
  if (TEMP == 0 ) {
    Output = 0;
    analogWrite(RELAY, 0);
    lcd.clear(); // LIMPIA LA PANTALLA
    lcd.setCursor(0, 0);
    lcd.print("Falla sensor DHT");

  } else {

    Input = TEMP;
    myPID.Compute();
    analogWrite(RELAY, Output);
    int valorFAN = 0;
    if (HUM < 50 ) {
      valorFAN = 255;
    } else {
      valorFAN = 120;
    }
    analogWrite(FAN, valorFAN);

    //mostrar en el puerto seril
    // Tempera hambiente.

    int val;//Crea una variable entera
    int temp;//Variable de temperatura = temp
    val = analogRead(A0); //Lee el valor del pin analogo 0 y lo mantiene como val
    temp = Thermister(val); //Realiza la conversión del valor analogo a grados Celsius
 
    //Serial.print("temperatura: ");
    //Serial.print(TEMP);
    //Serial.print(" Salida: ");
    //Serial.println(Output);
    // Serial.print("temperatura Hambiente: ");
    //Serial.println(temp);//Escribe la temperatura en el monitor serial

    //Mostrar en pantalla
    
    lcd.setCursor(0, 0);
    lcd.write((byte)1);
    lcd.print("A:");
    lcd.print(temp);
    lcd.write((byte)2);
    lcd.print("C");
    lcd.setCursor(11, 0);
    lcd.write((byte)0);
    lcd.print(HUM);
    lcd.print("%");
    lcd.setCursor(3, 1);
    lcd.write((byte)1);
    lcd.print("I: ");
    lcd.print(TEMP);
    lcd.write((byte)2);
    lcd.print("C");
    
  }

  rotacion ();
}

void rotacion ()  {

  time1 = millis();
  if ( time1 - previoTime > tROTARHUEVOS ) {
    previoTime = time1;
    //Serial.println(statusServo ? SERVOa1 : SERVOa2);
    servo.write(statusServo ? SERVOa1 : SERVOa2 );
    statusServo = !statusServo;

  }
}

double Thermister(int RawADC) {
  double Temp;
  Temp = log(((10240000 / RawADC) - 10000));
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp )) * Temp );
  Temp = Temp - 283.15;// Converierte de Kelvin a Celsius
  //Para convertir Celsius a Farenheith esriba en esta linea: Temp = (Temp * 9.0)/ 5.0 + 32.0;
  return Temp;
}
