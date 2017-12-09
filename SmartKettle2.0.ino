//Librerías
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>

//Variables
#define PinTem 6                        //Pin Temperatura
#define PinBomba 8                      //Pin Bomba de Agua
#define PinRele 4                       //Pin Relé de Estado Sólido
#define TiempoCiclo 1000                //Tiempo de Ciclo PID

double Setpoint, Input, Output;         //Variable PID
double Kp = 10, Ki = 3, Kd = 400;
unsigned long respuestaUltimaTemperatura = 0;
unsigned long lastPIDCalculation = 0;
float prevTemperature = -9999.0;
int Encendido = 0; // 0 para apagado, 1 para encendido
float TempActual = 0;
int TempFinal = 100;
const int tiempoChico = 30000;          //Tiempo de funcionamiento de Bomba de Agua
const int tiempoGrande = 35000;
char taza;

OneWire ourWire(PinTem);                //Se establece el pin declarado como bus para la comunicación OneWire
DallasTemperature sensors(&ourWire);    //Se llama a la librería DallasTemperature
SoftwareSerial BT1(12,11);              //Comunición Bluetooth (RX, TX)
PID miPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);  //Definir PID

void setup() {
  miPID.SetOutputLimits(0, TiempoCiclo); //Limites del Ciclo de PID
  miPID.SetSampleTime(TiempoCiclo);
  miPID.SetMode(AUTOMATIC);              //Ejecuta PID Automáticamete
  sensors.begin();                       //Inicia los sensores
  BT1.begin(9600);                       //Inicia puertos Serial de comunicación
  Serial.begin(9600);
  pinMode(PinRele, OUTPUT);
  pinMode(PinBomba, OUTPUT);
  pinMode(13,OUTPUT);
  digitalWrite(PinRele, LOW);
}

void loop() {

  //COMUNICACIÓN ARDUINO - APP
    if(BT1.available()==6){ //Lee el string que manda la APP y codifica el mensaje
      Encendido = 1;
      char uno = BT1.read();
      char datos[6];
      datos[0] = BT1.read();
      datos[1] = BT1.read();
      datos[2] = BT1.read();
      datos[3] = BT1.read();
      datos[4] = BT1.read();
      datos[5] = '\0';
      taza = datos[4];
      String datosfinal = String(datos);
      datosfinal.remove(3);
      TempFinal = datosfinal.toInt();
      }
    if(BT1.available()==5){
      Encendido = 1;
      char uno = BT1.read();
      char datos[5];
      datos[0] = BT1.read();
      datos[1] = BT1.read();
      datos[2] = BT1.read();
      datos[3] = BT1.read();
      datos[4] = '\0';
      taza = datos[3];
      String datosfinal = String(datos);
      datosfinal.remove(2);
      TempFinal = datosfinal.toInt();
      Serial.println(TempFinal);
    }
    if(BT1.available()==4 ){
      Encendido = 1;
      char uno = BT1.read();
      char datos[4];
      datos[0] = BT1.read();
      datos[1] = BT1.read();
      datos[2] = BT1.read();
      datos[3] = '\0';
      taza = datos[2];
      String datosfinal = String(datos);
      datosfinal.remove(1);
      TempFinal = datosfinal.toInt();
    }
    if(BT1.available()==3){
      Encendido = 0;
    }
  //Ejecuta PID
  if (millis() - respuestaUltimaTemperatura >= TiempoCiclo) {
    temperatura();
    Input = (double)TempActual;
    Setpoint = TempFinal;
    miPID.Compute();
    lastPIDCalculation = millis();
    respuestaUltimaTemperatura = millis();
  }
  flujo();
  calentar();
}

//Definición de funciones
void calentar(){
  if(Encendido == 0){
    digitalWrite(PinRele,LOW);
    digitalWrite(13,HIGH);
    delay(300);
    digitalWrite(13,LOW);
    delay(300);
  }
  else{
    if ((millis() <= (lastPIDCalculation + Output)) || (Output == TiempoCiclo)){ //Encendido ON
        digitalWrite(PinRele, HIGH);
        digitalWrite(13,HIGH);
        delay(1000);
        digitalWrite(13,LOW);
        delay(1000);
    }
    else { //Encendido OFF
      digitalWrite(PinRele, LOW);
    }
  }
}

void temperatura(){                      //Mide la temperatura del agua
  sensors.requestTemperatures();         //Prepara el sensor para la lectura
  TempActual=sensors.getTempCByIndex(0); //Lee la temperatura
  BT1.println(TempActual);               //Avisa temperatura actual al Bluetooth
}

void flujo(){                            //Permite el flujo de agua del estanque exterior al interior
  if (taza == '1'){                      //Fluyen 200mL
    digitalWrite(PinBomba,HIGH);
    delay(tiempoChico);
    digitalWrite(13,HIGH);
    digitalWrite(PinBomba,LOW);
    taza = '0';
  }
  if (taza == '2'){                     //Fluyen 300mL
    digitalWrite(PinBomba,HIGH);
    delay(tiempoGrande);
    digitalWrite(PinBomba,LOW);
    taza = '0';
  }
}
