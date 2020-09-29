/*---------------------------------------------------
Wallbox Webserver for ESP32 


by Jan Schuster - free for anyone



---------------------------------------------------*/
#define SC_W 160  //screen width in pixels
#define SC_H 128  //screen Hight in pixels
#define _Digole_Serial_I2C_  //To tell compiler compile the special communication only, 
#include <DigoleSerial.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "time.h"
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP_WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <WiFiUdp.h>
DigoleSerialDisp mydisp(&Wire,'\x29');  //I2C:Arduino UNO: SDA (data line) is on analog input pin 4, and SCL (clock line) is on analog input pin 5
#include <ADS1115_WE.h>
#include <Wire.h>
#define ESP_getChipId()   ((uint32_t)ESP.getEfuseMac())

////////////////////////////////////////////////////////////////////////////////////////
// Globale Variablen für Configuration
////////////////////////////////////////////////////////////////////////////////////////
String deviceID;
WiFiClient espClient;
WiFiUDP Udp;
unsigned int localUdpPort = 4210;
// WLAN where we want to be client - put your SSID/password here
char* uissid = "schuschi@konfig";
char* password = "schuschi17";
// SSID and PW for your Router
String Router_SSID;
String Router_Pass;

String SensorName="Default";      // Name des Sensors
/////////////////////////////////////////////////////////////////////////
//                             FEMS Rest API (Daten aus der Hausanlage)
////////////////////////////////////////////////////////////////////////
byte fSOC;                            //Aktueller State of Charge der Batterie 
float fPower;                          // Power von Solaranlage  
byte setSOC=15;                       // Defaultwert SOC aus Config.html
String setIP="192.168.1.24";          // Defaultwerte für FEMS REST API
/////////////////////////////////////////////////////////////////////////
//                             ntp timestamp
////////////////////////////////////////////////////////////////////////
time_t Startup_time, Start_Load, cur_time, Stop_Load;              // Startup Time des Sensors, Start_load Car , Current time
unsigned long check_sum;          // Distance to Check Summertime
unsigned long ulcurrentmillis;
unsigned long last_pub;           // Distance to MQTT Publish
unsigned long refresh_Home;
unsigned long ulReconncount;       // how often did we connect to WiFi
unsigned long ulMeasCount=0;    // values already measured
unsigned long ulNoMeasValues=0; // size of array
unsigned long ulMeasDelta_ms;   // distance to next meas time
unsigned long ulNextMeas_ms;    // next meas time
unsigned long *pulTime;         // array for time points of measurements
int *pfSOC,*pfSolar;             // SOC Battery; PowerSolar  
unsigned long ulReqcount;       // how often has a valid page been requested

String sLine13, sLine12;
const unsigned char homeimage[] PROGMEM = {
0,0,0,0,0,36,45,41,77,77,77,73,77,77,78,78,78,77,77,44,36,0,0,0,0
,0,0,0,0,0,45,46,42,42,78,42,42,42,78,79,79,83,114,44,76,76,0,0,0,0
,0,0,0,0,77,110,78,78,114,110,78,78,114,111,115,115,115,44,81,187,76,72,0,0,0
,0,0,0,72,78,42,42,78,78,42,42,78,78,78,115,115,77,44,219,219,182,40,36,0,0
,0,0,40,77,78,74,78,78,78,78,78,78,42,79,115,82,44,150,219,219,219,113,76,0,0
,0,36,77,78,78,78,114,78,78,78,114,78,78,115,115,44,113,219,255,255,219,219,76,40,0
,0,77,78,42,42,78,46,42,42,78,78,42,46,79,77,76,219,255,255,255,255,219,182,44,0
,40,77,77,78,78,78,78,78,78,78,78,78,78,82,44,182,223,255,255,255,255,219,219,77,72
,0,77,150,145,113,113,77,77,77,77,45,77,77,44,113,219,255,255,255,255,255,255,219,73,0
,0,109,219,219,219,219,219,219,219,219,219,186,182,150,219,255,255,255,255,255,255,255,219,36,0
,0,73,255,255,255,255,255,255,255,223,223,223,219,223,255,255,255,251,218,182,182,255,255,36,0
,0,73,255,255,223,109,141,141,146,182,255,255,255,255,255,255,219,68,68,100,145,255,255,36,0
,0,73,255,255,255,109,146,109,146,150,255,255,255,255,255,255,219,100,100,100,146,255,255,0,0
,0,73,255,255,255,109,146,109,146,150,255,255,255,255,255,255,219,100,100,100,146,255,255,0,0
,0,37,255,255,255,105,109,109,109,146,255,255,255,255,255,255,219,100,100,100,146,255,255,0,0
,0,37,255,255,255,109,146,109,146,150,255,255,255,255,255,255,219,100,100,68,146,255,255,0,0
,0,37,255,255,255,105,114,109,146,150,255,255,255,255,255,255,219,100,100,68,178,255,255,0,0
,0,36,255,255,255,178,145,109,109,109,255,255,255,255,255,255,214,100,100,68,182,255,219,0,0
,0,36,255,255,255,255,219,219,219,219,255,255,255,255,255,255,214,100,100,68,182,255,219,0,0
,0,36,255,255,255,255,255,255,255,255,255,255,255,255,255,255,214,100,100,68,182,255,219,0,0
,0,36,255,255,255,255,255,255,255,255,255,255,255,255,255,255,182,100,100,68,182,219,219,0,0
,0,36,219,223,255,255,255,255,255,255,255,255,255,255,255,255,182,100,68,68,182,219,182,0,0
,0,36,109,146,182,219,219,223,255,255,255,255,255,255,255,255,182,104,141,146,146,109,36,0,0
,0,0,0,0,36,73,109,146,146,182,219,219,223,255,219,219,182,146,109,73,36,0,0,0,0
,0,0,0,0,0,0,0,0,36,36,73,109,146,182,146,109,73,36,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,36,36,0,0,0,0,0,0,0,0,0,0
};
const unsigned char carimage[] PROGMEM = {
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,73,109,109,109,146,146,146,146,146,146,146,146,146,146,109,109,73,73,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,36,73,36,73,36,36,36,36,36,36,36,36,36,36,73,146,36,36,36,36,0,36,36,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,36,73,73,182,146,36,36,73,36,73,36,0,0,0,73,146,73,36,73,36,36,36,36,36,109,36,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,36,73,73,146,219,182,36,36,109,36,36,0,36,0,0,0,36,146,36,36,146,109,36,0,36,36,36,182,73,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,146,73,73,146,182,182,109,36,36,73,0,0,0,0,36,36,36,73,146,36,146,186,182,146,36,73,109,182,219,219,182,73,0
,0,0,0,0,0,0,0,0,36,36,73,146,182,182,182,182,182,182,182,146,146,146,146,146,146,109,109,109,109,146,109,73,73,109,146,182,219,219,219,219,182,182,182,182,0
,0,0,0,0,73,73,146,219,219,219,219,219,219,219,219,219,219,219,182,182,218,219,219,219,219,219,219,219,219,219,186,150,146,182,218,182,182,182,182,182,182,182,182,182,109
,0,0,36,109,182,182,219,219,219,219,182,182,182,182,182,182,182,182,182,186,182,109,73,73,146,182,182,182,182,182,182,182,182,182,182,182,182,182,182,182,150,182,36,36,109
,0,0,109,109,182,186,186,182,182,182,182,182,182,182,182,182,182,182,182,73,36,109,109,109,182,182,182,109,146,182,146,146,182,182,146,146,146,146,146,146,146,109,36,36,73
,0,73,109,146,182,182,182,182,182,182,182,182,182,182,182,182,182,146,73,73,146,146,146,146,182,182,73,36,0,146,146,113,146,146,146,146,146,146,146,146,146,109,36,36,73
,36,182,182,182,182,182,182,182,182,182,182,182,182,182,182,182,219,109,109,146,182,182,182,182,182,146,36,73,36,73,146,146,146,146,146,146,146,146,146,146,146,73,36,36,73
,73,150,182,182,182,182,182,182,182,182,182,182,182,182,218,219,219,219,219,223,219,219,214,209,182,36,36,0,36,73,146,146,146,146,146,146,146,182,182,182,182,72,36,36,36
,109,109,146,146,146,146,146,146,146,146,146,146,146,146,150,150,182,182,182,182,219,182,182,182,150,36,36,36,0,73,109,182,150,146,182,182,182,182,182,146,146,36,36,36,0
,36,36,36,36,36,73,73,73,109,109,109,109,109,109,109,109,77,109,109,146,146,146,146,146,113,36,36,36,32,36,73,146,182,182,182,146,146,109,73,36,36,36,36,36,0
,36,36,73,0,0,0,0,0,36,0,0,0,0,0,36,146,36,36,36,36,36,73,150,146,109,36,73,36,0,36,73,146,146,109,73,36,36,109,109,0,0,36,36,36,0
,36,146,109,73,0,0,0,0,0,0,0,36,0,0,73,182,73,0,36,36,36,36,182,146,109,36,36,36,36,73,36,36,0,0,0,0,0,0,0,0,0,36,36,36,0
,0,36,73,109,109,109,73,73,73,73,73,73,73,73,146,182,150,109,109,109,109,146,146,146,73,36,36,0,36,109,0,0,0,0,0,36,36,73,73,73,36,0,0,0,0
,0,0,0,0,36,0,0,0,0,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,73,36,36,36,73,73,73,73,36,0,0,0,0,0,0,0,0
,0,0,0,36,146,146,109,73,73,36,36,36,0,0,0,0,0,0,0,0,0,0,0,36,36,73,73,73,73,109,73,73,36,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,36,36,36,36,73,73,73,36,73,36,36,36,36,36,36,36,36,36,36,36,36,36,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0

};
//Ladebox spezifische Parameter
// Falls der folgende Wert zu true geändert wird, beendet die Ladebox einen Ladevorgang nicht, wenn das E-Auto eine Belüftung anfordert. 
// Dies ist eigentlich nur für (alte) Elektrofahrzeuge mit Bleiakkumulatoren relevant und darf nur aktiviert werden, wenn die Ladebox ausschließlich im Freien genutzt wird.
#define BELUEFTUNG false

// Wenn der folgende Wert bei false belassen wird, erwartet die Ladebox zunächst den Status +9 V vom E-Auto, bevor sie das PWM-Signal aktiviert. 
// Erst dann kann das E-Auto auf +6 V (oder +3 V) wechseln und somit den Ladevorgang starten. 
// Falls der folgende Wert zu true geändert wird, startet die Ladebox einen Ladevorgang auch dann, wenn bereits ohne PWM-Signal der Status +6 V gemeldet wird.
// Dies ist beispielsweise bei vielen einfachen Typ2-Adaptern (z.B. Typ2 auf Schuko) oder ggf. bei älteren oder umgebauten Elektrofahrzeugen notwendig.
#define ADAPTERKOMPATIBILITAET false

unsigned long Millisekunden, gespeicherte_Zeit;
unsigned long letzte_Messung, CP_Messung;
bool Ergebnis_korrekt=false;
byte Betriebsphase = 0;
float Progress=0;
int16_t CP_Plus [10], CP_Minus [10];  // Arrays für die 100-fache Messung der positiven und negativen Spannung am CP-Pin
byte CP_Status[9];  // Zählt, wie viele Messwerte von welcher Kategorie es jeweils gibt.
byte Status_Fahrzeug = 255; // Bei erfolgreicher Messung des Fahrzeugstatus wird dieser hier gespeichert. 
                           //  Mögliche Werte sind PLUS_12_V, PLUS_9_V, PLUS_6_V, PLUS_3_V und MINUS_12_V. 255 bedeutet: Status wurde noch nicht erkannt.
#define START_STROMSTAERKE_DREIPHASIG 0  // gibt an, welche Stromstärke bei dreiphasigem Anschluss voreingestellt ist (Ladevorgang startet nicht selbstständig). Hier 16 A.
    
// Durch die folgenden Werte werden die ADC-Messwerte dividiert, um die Spannung am CP-Pin zu erhalten.

#define CP_TIMEOUT 1500  // gibt an, nach welcher Zeit (in Millisekunden) ein Ladevorgang abgebrochen wird, wenn der Status des E-Autos nicht ermittelt werden kann.
byte Pulsweite, letzte_Pulsweite;  // Speichert die aktuelle Pulsweite des CP-Signals. 0 bedeutet 0 %, 255 bedeutet 100 %.
byte Ladekabel_max;  // Speichert die maximal mögliche Pulsweite aufgrund der Belastbarkeit des Ladekabels. 255 bedeutet: kein Ladekabel angeschlossen.
byte Stromstaerke=1; // Wertebereich: 0 bis 4. Speichert die eingestellte Ladestromstärke. Dient als Index für PULSWEITE und O_LED_STROM.
// Folgendes Array legt fest, welche fünf verschiedenen Ladestromstärken ausgewählt werden können. 
// Die Werte können individuell angepasst werden, müssen aber in aufsteigender Reihenfolge sortiert sein.
// Es handelt sich um 8-Bit-Werte, d.h. 0 bedeutet 0 % Pulsweite, 255 bedeutet 100 % Pulsweite.
//10%=6A; 16%=9,6A; 25%=15A; 30%=18A; 40%=24A; 50%=30A; 60%=36A;
const byte PULSWEITE [5] = {   255*10/100,   255*16/100, 255*25/100 , 255*30/100,  255*40/100};    // 6A, 9,6A, 15A, 18A, 24A
//PWM to Spannung 1000 hz = 1 ms
// Um=Uaus+((Uein−Uaus)⋅tein/(tein+taus))  Uaus=-12V Uein=12V  tein=1000*Pulsweite/255 taus=1000-tein=1000-(1000*Pulsweite/255) 
const uint16_t  FAKTOR_PLUS[5]={9,17,29,34,44};
const byte STROM[5] = { 6, 10 , 15 , 18 , 24 };
byte Fehlercode = 0;  // 0 bedeutet: kein Fehler. Andere mögliche Werte sind die mit F_ gekennzeichneten Konstanten (siehe oben unter DEFINITIONEN).

// Folgenden Wert zu true ändern, um Statusinformationen über die serielle Schnittstelle ausgeben zu lassen. Dies reduziert allerdings
// die Reaktionsgeschwindigkeit und Sicherheit der Ladebox und sollte daher nur zur Fehlersuche verwendet werden.
#define DEBUG false
// Im Folgenden die Pinbelegung:
// Ausgänge sind mit O_ (für Output) gekennzeichnet.
// Digitale Eingänge sind mit I_ (für Input) gekennzeichnet.
// Analoge Eingänge (ADC) sind mit A_ gekennzeichnet.
#define O_WAKEUP          18  // Ansteuerung des Relais für CP Signalunterbrechung (Fahrzeuge schlafen ein) dient zum aufwecken des Fahrzeugs
#define O_LADUNG          19  // Ansteuerung der Relais bzw. des Schütz zur Stromzufuhr zum Elektrofahrzeug
#define O_CP_SIGNAL       23  // Ausgabe des ±12-V-PWM-Signals am CP-Pin, durch welches dem Elektrofahrzeug die mögliche Ladestromstärke mitgeteilt wir
// setting PWM properties
#define freq            1000
#define CP_Channel      0
#define resolution      8
// Mithilfe der folgenden Fehlercodes kann die Fehlerursache identifiziert werden. So oft blinkt die Fehler-LED (es wird immer nur der jeweils erste erkannte Fehler angezeigt).
#define F_TEMP_MAX         1  // Höchsttemperatur überschritten
#define F_TEMP_MIN         2  // Mindesttemperatur unterschritten (vermutlich Temperatursensor defekt oder Auswertung fehlerhaft)
#define F_VERRIEGELUNG     3  // Verriegelung funktioniert nicht
#define F_PP_UNDEFINIERT   4  // PP-Widerstand entspricht keinem definierten Wert oder hat sich zu einem anderen verändert
#define F_PP_FEHLT         5  // PP-Widerstand fehlt (wenn über CP ein Fahrzeug erkannt wurde)
#define F_CP_TIMEOUT       6  // Der Status des E-Autos konnte innerhalb von CP_TIMEOUT nicht ermittelt werden
#define F_CP_UNDEFINIERT   7  // Keinen definierten Status (des E-Autos) erkannt
#define F_CP_SPANNUNG      8  // Spannung am CP-Pin größer als +12 V oder kleiner als -12 V gemessen
#define F_CP_DIODE         9  // Diodenfehler (beim E-Auto)
#define F_CP_KURZSCHLUSS  10  // CP-Pin mit GND bzw. PE verbunden
#define F_NOTABBRUCH      11  // Elektroauto ist plötzlich nicht mehr angeschlossen und hat vorher nicht den Ladevorgang beendet
#define F_SCHALTER        12  // Der Schalter zur Wahl zwischen ein- und dreiphasiger Ladung wurde während des Betriebs umgestellt
#define F_BELUEFTUNG      13  // Elektroauto fordert Belüftung an (wird natürlich nur als Fehler gewertet, wenn BELUEFTUNG als false definiert ist)
#define F_WIFI            14  // keine WIFI Verbindung
// Kategorien für Messwerte der positiven Spannung an CP (Index für Array CP_Status und Wert für Status_Fahrzeug):
#define PLUS_12_V           0
#define PLUS_9_V            1
#define PLUS_6_V            2
#define PLUS_3_V            3
#define PLUS_UNDEFINIERT    4
#define NICHT_POSITIV       5

// Kategorien für Messwerte der negativen Spannung an CP (Index für Array CP_Status und Wert für Status_Fahrzeug):
#define MINUS_12_V          6
#define MINUS_UNDEFINIERT   7
#define NICHT_NEGATIV       8
// Weitere Kategorie für zu hohe Messwerte der positiven und negativen Spannung an CP (Index für Array CP_Status):
#define ZU_HOHE_SPANNUNG    9



byte Ergebnis;
int freeheap;
////////////////////////////////////////////////////////////////////////////////////////

// storage for Measurements; keep some mem free; allocate remainder
#define KEEP_MEM_FREE 185000
#define MEAS_SPAN_H 25

// Create an instance of the server on Port 80
WiFiServer server(80);
//////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Sensor Settings
//////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////
//Analog digital Wandler ADS1115
//////////////////////////////
ADS1115_WE ads = ADS1115_WE();
//Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

int toprint = 0;
unsigned long ulNextPrint_ms;        // next display change time
unsigned long ulPrintDelta_ms =1500; // Display aktualisierung
// needed to avoid link error on ram check

void FreeHEAP() {
  if ( ESP.getFreeHeap() < freeheap ) {
    if ( ( freeheap != KEEP_MEM_FREE) ) {
       Serial.print("Memory leak detected! old free heap = ");
       Serial.print(freeheap);
       Serial.print(", new value = ");
       Serial.println(ESP.getFreeHeap());
    }
    freeheap = ESP.getFreeHeap();
  }
}
String uint64ToString(uint64_t input) {
  String result = "";
  uint8_t base = 10;

  do {
    char c = input % base;
    input /= base;

    if (c < 10)
      c +='0';
    else
      c += 'A' - 10;
    result = c + result;
  } while (input);
  return result;
}
/////////////////////
// the setup routine
/////////////////////
void setup() 
{
  Wire.begin();
   // configure LED PWM functionalitites
  ledcSetup(CP_Channel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(O_CP_SIGNAL, CP_Channel);
  // setup globals
  pinMode(O_LADUNG, OUTPUT); // Setzt den Digitalpin 19 als Outputpin
  pinMode(O_WAKEUP, OUTPUT); // Setzt den Digitalpin 18 als Outputpin
  ulReqcount=0; 
  ulReconncount=0;
 // start serial
  Serial.begin(115200);
  mydisp.begin();
  mydisp.clearScreen();
  mydisp.setColor(191);
  mydisp.drawStr(0, 0, "....boot");
  /* Set the voltage range of the ADC to adjust the gain
   * Please note that you must not apply more than VDD + 0.3V to the input pins!
   * 
   * ADS1115_RANGE_6144  ->  +/- 6144 mV
   * ADS1115_RANGE_4096  ->  +/- 4096 mV
   * ADS1115_RANGE_2048  ->  +/- 2048 mV (default)
   * ADS1115_RANGE_1024  ->  +/- 1024 mV
   * ADS1115_RANGE_0512  ->  +/- 512 mV
   * ADS1115_RANGE_0256  ->  +/- 256 mV
   */
   /* Set the conversion rate in SPS (samples per second)
   * Options should be self-explaining: 
   * 
   *  ADS1115_8_SPS 
   *  ADS1115_16_SPS  
   *  ADS1115_32_SPS 
   *  ADS1115_64_SPS  
   *  ADS1115_128_SPS (default)
   *  ADS1115_250_SPS 
   *  ADS1115_475_SPS 
   *  ADS1115_860_SPS 
   */
  if(!ads.init()){
     Serial.println("ADS1115 not connected!");
  }   
  ads.setVoltageRange_mV(ADS1115_RANGE_2048); //comment line/change parameter to change range 
  ads.setCompareChannels(ADS1115_COMP_0_GND);
  //ads.setCompareChannels(ADS1115_COMP_1_GND);
  //ads.setCompareChannels(ADS1115_COMP_2_GND);
  //ads.setCompareChannels(ADS1115_COMP_3_GND);
  ads.setMeasureMode(ADS1115_CONTINUOUS);
  ads.setConvRate(ADS1115_64_SPS);

  
  uint32_t free=system_get_free_heap_size() - KEEP_MEM_FREE;
  Serial.print("freeHeap:");
  Serial.println(free);
  ulNoMeasValues = free / (sizeof(int)*2+sizeof(unsigned long));  // SOC Power Solar + time 
  Serial.print("number Messuresvalues:");
  Serial.println(ulNoMeasValues);
  pulTime = new unsigned long[ulNoMeasValues];
  pfSOC = new int[ulNoMeasValues];
  pfSolar = new int[ulNoMeasValues];
  
  if (pulTime==NULL || pfSOC==NULL|| pfSolar==NULL)
  {
    ulNoMeasValues=0;
    Serial.println("Error in memory allocation!");
  }
  else
  {
    Serial.print("Allocated storage for ");
    Serial.print(ulNoMeasValues);
    Serial.println(" data points.");
    
    float fMeasDelta_sec = MEAS_SPAN_H*3600./ulNoMeasValues;
    ulMeasDelta_ms = ( (unsigned long)(fMeasDelta_sec+0.5) ) * 1000;  // round to full sec
    Serial.print("Measurements will happen each ");
    Serial.print(ulMeasDelta_ms);
    Serial.println(" ms.");
    
    ulNextMeas_ms = millis()+ulMeasDelta_ms;
  }
  delay(random(2000));
  WiFiStart();
  //espClient.setNoDelay(true);
  //espClient.setSync(true);
  
  FreeHEAP();
  freeheap = ESP.getFreeHeap();
  // Start the server
  server.begin();
  Serial.print("Server listening on Port 80: ");
  // Print the IP address
  Serial.println(WiFi.localIP());
  homescreen();
  refresh_Home=millis()+(1000*3600);  //Homescreen refresh jede Stunde
  letzte_Messung = millis();
  digitalWrite( O_WAKEUP, HIGH );  // Schalte CP Signal zum Auto an.
}


///////////////////
// (re-)start WiFi
///////////////////
void WiFiStart()
{
  unsigned long startedAt = millis();
  deviceID =  uint64ToString(ESP.getEfuseMac());     // IoT thing device ID - unique device id in our project
  freeheap = ESP.getFreeHeap();
  ulReconncount++;
  //uissid = "schuschi@konfig";
  //password = "schuschi17";
  mydisp.setColor(191);
  mydisp.drawStr(0, 0, "....Connecting");
  ESP_WiFiManager wifiManager;  
  // We can't use WiFi.SSID() in ESP32as it's only valid after connected. 
  // SSID and Password stored in ESP32 wifi_ap_record_t and wifi_config_t are also cleared in reboot
  // Have to create a new function to store in EEPROM/SPIFFS for this purpose
  Router_SSID = wifiManager.WiFi_SSID();
  Router_Pass = wifiManager.WiFi_Pass();
  
  //Remove this line if you do not want to see WiFi password printed
  Serial.println("Stored: SSID = " + Router_SSID + ", Pass = " + Router_Pass);
  
  //Check if there is stored WiFi router/password credentials.
  //If not found, device will remain in configuration mode until switched off via webserver.
  Serial.print("Opening configuration portal.");
  mydisp.clearScreen();
  mydisp.setColor(191);
  mydisp.drawStr(0, 0, "open Portal(120s)");
  
  if (Router_SSID != "")
  {
    wifiManager.setConfigPortalTimeout(120); //If no access point name has been previously entered disable timeout.
    Serial.println("Timeout 120s");
  }
  else
    Serial.println("No timeout");

  // SSID to uppercase 
  //ssid.toUpperCase();  

  if (!wifiManager.startConfigPortal((const char *) uissid, password)) 
    Serial.println("Not connected to WiFi but continuing anyway.");
  else 
    Serial.println("WiFi connected...");
  // For some unknown reason webserver can only be started once per boot up 
  // so webserver can not be used again in the sketch.
  #define WIFI_CONNECT_TIMEOUT        30000L
  #define WHILE_LOOP_DELAY            200L
  #define WHILE_LOOP_STEPS            (WIFI_CONNECT_TIMEOUT / ( 3 * WHILE_LOOP_DELAY ))
  startedAt = millis();  
  while ( (WiFi.status() != WL_CONNECTED) && (millis() - startedAt < WIFI_CONNECT_TIMEOUT ) )
  {   
   
    WiFi.mode(WIFI_STA);
    WiFi.persistent (true);
    // We start by connecting to a WiFi network
    
    Serial.print("Connecting to ");
    Serial.println(Router_SSID);
    WiFi.begin(Router_SSID.c_str(), Router_Pass.c_str());
   

    int i = 0;
    while((!WiFi.status() || WiFi.status() >= WL_DISCONNECTED) && i++ < WHILE_LOOP_STEPS)
    {
      delay(WHILE_LOOP_DELAY);
    }    
  }
  Serial.println("");
  Serial.println("WiFi connected");
  mydisp.setColor(191);
  mydisp.drawStr(0, 0, "WIFI Connected");
  
 ///////////////////////////////
  // connect to NTP and get time
  ///////////////////////////////
  configTime(0,0, "pool.ntp.org", "time.nist.gov");
  setenv("TZ", "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00", 1);
  Serial.println("\nWaiting for time");
  while(time(nullptr) <= 100000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("time done");
  Startup_time = time(nullptr);
  Serial.println(ctime(&Startup_time));
  //Serial.println(Startup_time);
  mydisp.setColor(191);
  mydisp.drawStr(0, 0, "WIFI Time done");
  freeheap = ESP.getFreeHeap();
}

/////////////////////////////////////
// make html table for measured data
/////////////////////////////////////
unsigned long MakeTable (WiFiClient *pclient, bool bStream)
{
  unsigned long ulLength=0;
  
  // here we build a big table.
  // we cannot store this in a string as this will blow the memory   
  // thus we count first to get the number of bytes and later on 
  // we stream this out
  if (ulMeasCount==0) 
  {
    String sTable = "Noch keine Daten verf&uuml;gbar.<BR>";
    if (bStream)
    {
      pclient->print(sTable);
    }
    ulLength+=sTable.length();
  }
  else
  { 
    unsigned long ulEnd;
    if (ulMeasCount>ulNoMeasValues)
    {
      ulEnd=ulMeasCount-ulNoMeasValues;
    }
    else
    {
      ulEnd=0;
    }
    
    String sTable;
    sTable = "<table style=\"width:100%\"><tr><th>Zeit / MEZ</th><th>SoC (%)</th><th>Solar kW</th></tr>";
    sTable += "<style>table, th, td {border: 2px solid black; border-collapse: collapse;} th, td {padding: 5px;} th {text-align: left;}</style>";
    for (unsigned long li=ulMeasCount;li>ulEnd;li--)
    {
      unsigned long ulIndex=(li-1)%ulNoMeasValues;
      sTable += "<tr><td>";
      struct tm * timeinfo;
      time_t helper = pulTime[ulIndex];
      timeinfo = localtime(&helper); 
      int year = timeinfo->tm_year+1900;
      int month = timeinfo->tm_mon+1;   
      int day = timeinfo->tm_mday;    
      int hour = timeinfo->tm_hour;
      int minutes = timeinfo->tm_min;
      int seconds = timeinfo->tm_sec;
      String sTime=String(hour)+":"+String(minutes)+":"+String(seconds)+" - "+String(day)+"."+String(month)+"."+String(year);
        
      sTable += sTime;
      sTable += "</td><td>";
      sTable += pfSOC[ulIndex];
      sTable += "</td><td>";
      sTable += pfSolar[ulIndex];
      sTable += "</td></tr>";

      // play out in chunks of 1k
      if(sTable.length()>1024)
      {
        if(bStream)
        {
          pclient->print(sTable);
          //pclient->write(sTable.c_str(),sTable.length());
        }
        ulLength+=sTable.length();
        sTable="";
      }
    }
    
    // remaining chunk
    sTable+="</table>";
    ulLength+=sTable.length();
    if(bStream)
    {
      pclient->print(sTable);
      //pclient->write(sTable.c_str(),sTable.length());
    }   
  }
  
  return(ulLength);
}
  

////////////////////////////////////////////////////
// make google chart object table for measured data
////////////////////////////////////////////////////
unsigned long MakeList (WiFiClient *pclient, bool bStream)
{
  unsigned long ulLength=0;
  
  // here we build a big list.
  // we cannot store this in a string as this will blow the memory   
  // thus we count first to get the number of bytes and later on 
  // we stream this out
  if (ulMeasCount>0) 
  { 
    unsigned long ulBegin;
    if (ulMeasCount>ulNoMeasValues)
    {
      ulBegin=ulMeasCount-ulNoMeasValues;
    }
    else
    {
      ulBegin=0;
    }
    
    String sTable="";
    for (unsigned long li=ulBegin;li<ulMeasCount;li++)
    {
      // result shall be ['18:24:08 - 21.5.2015',21.10,49.00],
      unsigned long ulIndex=li%ulNoMeasValues;
      struct tm * timeinfo;
      time_t helper = pulTime[ulIndex];
      timeinfo = localtime(&helper); 
      int year = timeinfo->tm_year+1900;
      int month = timeinfo->tm_mon+1;   
      int day = timeinfo->tm_mday;    
      int hour = timeinfo->tm_hour;
      int minutes = timeinfo->tm_min;
      int seconds = timeinfo->tm_sec;
      String sTime=String(hour)+":"+String(minutes)+":"+String(seconds)+" - "+String(day)+"."+String(month)+"."+String(year);
      
      sTable += "['";
      sTable += sTime;
      sTable += "',";
      sTable += pfSOC[ulIndex];
      sTable += ",";
      sTable += pfSolar[ulIndex];
      sTable += "],\n";

      // play out in chunks of 1k
      if(sTable.length()>1024)
      {
        if(bStream)
        {
          pclient->print(sTable);
          //pclient->write(sTable.c_str(),sTable.length());
        }
        ulLength+=sTable.length();
        sTable="";
      }
    }
    
    // remaining chunk
    if(bStream)
    {
      pclient->print(sTable);
      //pclient->write(sTable.c_str(),sTable.length());
    } 
    ulLength+=sTable.length();  
  }
  
  return(ulLength);
}

// ADC1115 Lese Wert aus ADC mit ADS1115_CONTINUOUS
//
// !!!!!IMPORTANT: Wichtig ist das Settup des AD Wandlers! 
// Zu schnell eingestellt, wird bei niedrigem DutyCycle die Low Phase gemessen.
//
float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  //ads.setCompareChannels(channel);
  //ads.startSingleMeasurement();
  //while(ads.isBusy()){}
  ads.setCompareChannels(channel); //comment line/change parameter to change channel
  delay(10);
  voltage = ads.getResult_mV(); // alternative: getResult_mV for Millivolt
  return voltage;
}
bool CP_messen(){
  float cp_P_durchschnitt=0;
  bool Ergebnis_korrekt = true;  
  // Führe 10 Messungen der Spannungen am CP-Pin plus durch:
    for(byte index = 0; index < 10; index++) {
      CP_Plus[index] =  readChannel(ADS1115_COMP_0_GND); 
      if (CP_Plus[index]<0) CP_Plus[index]=0;
    }
   // Führe 10 Messungen der Spannungen am CP-Pin minus durch:
    for(byte index = 0; index < 10; index++) {
      CP_Minus[index]= readChannel(ADS1115_COMP_1_GND); 
      if (CP_Minus[index]<0) CP_Minus[index]=0.1;
    } 
  // Verarbeitung der CP-Messwerte:
  for(byte i = 0; i < 10; i++) CP_Status[i] = 0;  // Lösche die von der letzten Messung gespeicherten Daten.

  for(byte index = 0; index < 10; index++) {
    // Berechnung der positiven und negativen Spannung:
    float Spannung_Plus;
    if(Pulsweite==255)     Spannung_Plus= (float)(CP_Plus[index]/133.3); //165 war es auch schon :-(
    else if (Pulsweite==0) Spannung_Plus=0;
    else                   Spannung_Plus= (float)(CP_Plus[index]/FAKTOR_PLUS[Stromstaerke]);
    //Serial.print(" fSpannung+: "); 
    //Serial.println(Spannung_Plus);
    cp_P_durchschnitt +=Spannung_Plus;
    float Spannung_Minus = (float)CP_Minus[index]/165;   // CP_MINUS_FAKTOR
    //Serial.print(" fSpannung-: "); 
    //Serial.println(Spannung_Minus);
    debug( " +" + String(Spannung_Plus, 2) + " V, -" + String(Spannung_Minus, 2) + " V," );
    
    // Auswertung der positiven Spannung:
    if     ( Spannung_Plus > 13.5 )               CP_Status[ZU_HOHE_SPANNUNG]++;
    else if( Spannung_Plus > 11.2 )                      CP_Status[PLUS_12_V]++;
    else if( Spannung_Plus > 7.5 && Spannung_Plus < 10.5 ) CP_Status[PLUS_9_V]++;
    else if( Spannung_Plus > 4.6 && Spannung_Plus < 7.2 ) CP_Status[PLUS_6_V]++;
    else if( Spannung_Plus > 2.2 && Spannung_Plus < 3.8 ) CP_Status[PLUS_3_V]++;
    else if( Spannung_Plus < 0.1 )                   CP_Status[NICHT_POSITIV]++;
    else                                          CP_Status[PLUS_UNDEFINIERT]++;
    
    // Auswertung der negativen Spannung:
    if     ( Spannung_Minus > 12.5 ) CP_Status[ZU_HOHE_SPANNUNG]++;
    else if( Spannung_Minus > 11.2 )       CP_Status[MINUS_12_V]++;
    else if( Spannung_Minus < 0.1 )     CP_Status[NICHT_NEGATIV]++;
    else                            CP_Status[MINUS_UNDEFINIERT]++;
   }
  
  debug("\nAnzahl Werte pro Bereich: ");
  //for(byte i = 0; i < 10; i++) Serial.println( CP_Status[i] );
  debug("\n");
  // Falls zu viele Messwerte betragsmäßig über 12,5 V liegen, beende den Ladevorgang:
  if( CP_Status[ZU_HOHE_SPANNUNG] > 5 ) Ergebnis_korrekt=false; //Fehler( F_CP_SPANNUNG );
  // Falls zu viele Messwerte betragsmäßig unter 0,1 V liegen, wird vermutet, dass die Spannung bei 0 V liegt. Beende den Ladevorgang:
  if( CP_Status[NICHT_POSITIV] + CP_Status[NICHT_NEGATIV] > 17 ) Fehler( F_CP_KURZSCHLUSS );
  
  // Ermittle vorläufiges Ergebnis der Messung: Welcher Wert wurde am häufigsten gemessen?
  if( Pulsweite == 0 ) Ergebnis = MINUS_12_V;
  else if( CP_Status[PLUS_12_V] > CP_Status[PLUS_9_V]  && CP_Status[PLUS_12_V] > CP_Status[PLUS_6_V] && CP_Status[PLUS_12_V] > CP_Status[PLUS_3_V] ) Ergebnis = PLUS_12_V;
  else if(  CP_Status[PLUS_9_V] > CP_Status[PLUS_12_V] &&  CP_Status[PLUS_9_V] > CP_Status[PLUS_6_V] &&  CP_Status[PLUS_9_V] > CP_Status[PLUS_3_V] ) Ergebnis = PLUS_9_V;
  else if(  CP_Status[PLUS_6_V] > CP_Status[PLUS_12_V] &&  CP_Status[PLUS_6_V] > CP_Status[PLUS_9_V] &&  CP_Status[PLUS_6_V] > CP_Status[PLUS_3_V] ) Ergebnis = PLUS_6_V;
  else if(  CP_Status[PLUS_3_V] > CP_Status[PLUS_12_V] &&  CP_Status[PLUS_3_V] > CP_Status[PLUS_9_V] &&  CP_Status[PLUS_3_V] > CP_Status[PLUS_6_V] ) Ergebnis = PLUS_3_V;
  else Ergebnis = PLUS_UNDEFINIERT;
  
  //Debug:  
  if ( Ergebnis == PLUS_UNDEFINIERT){
   Serial.print("CP zu hoch anzahl: ");
   Serial.println(CP_Status[ZU_HOHE_SPANNUNG]);
   Serial.print("CP 12V anzahl: ");
   Serial.println(CP_Status[PLUS_12_V]); 
   Serial.print("CP 9V anzahl: ");
   Serial.println(CP_Status[PLUS_9_V]); 
   Serial.print("CP 6V anzahl: ");
   Serial.println(CP_Status[PLUS_6_V]); 
   Serial.print("CP undefined anzahl: ");
   Serial.println(CP_Status[PLUS_UNDEFINIERT]); 
   Serial.print("CP nicht negativ anzahl: ");
   Serial.println( CP_Status[NICHT_NEGATIV]); 
   Serial.print("Stromstärke: ");
   Serial.println(Stromstaerke);
   Serial.print("Faktor: ");
   Serial.println(FAKTOR_PLUS[Stromstaerke]);
   Serial.print("Pulsweite: ");
   Serial.println(Pulsweite);
   Serial.print("CP durchschnitt: ");
   Serial.println(cp_P_durchschnitt/10);
   Serial.print("vorläufiges Ergebnis(CP-Tabelle!!): ");
   Serial.println(Ergebnis);
   Serial.print("realwerte CP_PLUS[x]: ");
   for(byte i = 0; i < 10; i++) {Serial.print(CP_Plus[i]); Serial.print(" , ");}  // Lösche die von der letzten Messung gespeicherten Daten.
   Serial.println(" "); 
   }

  // Kein Messwert darf höher als das Ergebnis sein:
  Ergebnis_korrekt &= ( CP_Status[ZU_HOHE_SPANNUNG] < 1 );
  Ergebnis_korrekt &= ( CP_Status[PLUS_12_V] == 0 || Ergebnis == PLUS_12_V );
  Ergebnis_korrekt &= ( CP_Status[PLUS_9_V]  == 0 || Ergebnis == PLUS_9_V || Ergebnis == PLUS_12_V || Ergebnis == PLUS_UNDEFINIERT );
  Ergebnis_korrekt &= ( CP_Status[PLUS_6_V]  == 0 || ( Ergebnis != PLUS_3_V && Ergebnis != MINUS_12_V ) );
  Ergebnis_korrekt &= ( CP_Status[PLUS_3_V]  == 0 || Ergebnis != MINUS_12_V );
  Ergebnis_korrekt &= ( CP_Status[PLUS_UNDEFINIERT] == 0 || Ergebnis != MINUS_12_V );
  //debug( String( Ergebnis_korrekt ) + ", " );
  //Serial.print("Kein Messwert höher (1=i.o): "); 
  //Serial.println(Ergebnis_korrekt); 
  //Ermittle anhand einer Formel, wie viele Messwerte vom ermittelten Status es mindestens geben muss, damit das Ergebnis akzeptiert werden kann:
  //byte Mindestanzahl_Werte_Plus = ( ((float)Pulsweite * (float)Pulsweite / 1000.0 ) + ( (float)Pulsweite / 10.0 ))/10 ;
  byte Mindestanzahl_Werte_Plus = 7 ;
  //Serial.print("Mindestanzahl_Werte_Plus: ");
  //Serial.println(Mindestanzahl_Werte_Plus); 
  //Ergebnis_korrekt &= ( CP_Status[NICHT_NEGATIV] >= Mindestanzahl_Werte_Plus );
  //debug( String( Ergebnis_korrekt ) + ", " );
  //Serial.print("Ergebnis (nicht negativ): ");
  //Serial.println(Ergebnis_korrekt); 
  Ergebnis_korrekt &= ( CP_Status[Ergebnis]      >= Mindestanzahl_Werte_Plus );
  //debug( String( Ergebnis_korrekt ) + ", " );
  //Serial.print("Ergebnis gesamt: ");
  //Serial.println(Ergebnis_korrekt);  
  // Ermittle anhand einer Formel, wie viele Messwerte von -12 V es mindestens geben muss, damit das Ergebnis akzeptiert werden kann:
  //byte Pulsweite_N = 255 - Pulsweite;
  //byte Mindestanzahl_Werte_Minus = ( (float)Pulsweite_N * (float)Pulsweite_N / 1000.0 ) + ( (float)Pulsweite_N / 10.0 ) ;
  //byte Mindestanzahl_Werte_Minus = 8 ;
  //Ergebnis_korrekt &= ( CP_Status[NICHT_NEGATIV] >= Mindestanzahl_Werte_Minus );
  //debug( String( Ergebnis_korrekt ) + ", " );
  //Serial.print("Ergebnis mindestanzahl Minus: ");
  //Serial.println(Ergebnis_korrekt); 
  //if( CP_Status[MINUS_12_V]>0|| CP_Status[MINUS_12_V] < Mindestanzahl_Werte_Minus ) {
 
    // Falls die negativen Messwerte zwischen 0,1 V und -11,2 V liegen, wird ein Diodenfehler vermutet (und der Ladevorgang abgebrochen):
  //  if( Ergebnis_korrekt && CP_Status[MINUS_UNDEFINIERT] >= Mindestanzahl_Werte_Minus ){
  //    Fehler( F_CP_DIODE );
  //  }
  //  else{
  //    Ergebnis_korrekt = false;
  //    Serial.print("Ergebnis Minus<Mindestanzahl unklar diese IF: ");
  //    Serial.println(Ergebnis_korrekt); 
  //  }
  // } 
  //debug( String( Ergebnis_korrekt ) + "\n" );
  //Serial.print ("Ergebnis Prüfung (Variable Ergebnis_korrekt : ");
  //Serial.println (Ergebnis_korrekt);
  //Serial.print ("Ergebnis aus CP-Tabelle: ");
  //Serial.println (Ergebnis); 
  if( Ergebnis == PLUS_UNDEFINIERT ) Ergebnis_korrekt=false; 
 
  
  return (Ergebnis_korrekt);
}

/////////////
// main look
/////////////
void loop() 
{
  cur_time = time(nullptr);
  //Timer reset bei überlauf
  if (millis()<ulcurrentmillis){last_pub=0; check_sum=0; CP_Messung=0;} 
  ulcurrentmillis = millis();
  //Timer Summertime alle Stunde ob das notwendig ist???
  if (ulcurrentmillis>=check_sum){                   
    check_sum=ulcurrentmillis;
    check_sum +=600000;
    if (summertime()){
      cur_time=cur_time+3600;
    }else{
      //Serial.print("daylight saving time: ");
      //Serial.println(dst);
    }
  }
  //Timer Homescreen refresh
  if (millis()>refresh_Home){
     homescreen();
     String t=sLine12;
     print_Line12("init..");
     print_Line12(t);
     t=sLine13;
     print_Line13("init..");
     print_Line13(t);
     refresh_Home=millis()+(1000*3600);
  }
  // Ladekabel fest auf 85:
  
  Ladekabel_max =  85;
  
  
  //Timer CP Messung
  if (ulcurrentmillis>=CP_Messung||Pulsweite!=letzte_Pulsweite){
      //Serial.println("//Beginn CP_messen");
      //Serial.print("Pulsweite vor Messung: ");
      //Serial.println(Pulsweite);
      Ergebnis_korrekt=CP_messen();
      //Serial.print("Return CP_messen: ");
      //Serial.println(Ergebnis_korrekt);
      //Serial.println("End CP_messen//");
      //if(Pulsweite==0) CP_Messung=ulcurrentmillis+50;
      CP_Messung=ulcurrentmillis+(500);
      letzte_Pulsweite=Pulsweite; 
  }
  
  Millisekunden = millis();  // Rufe die vergangene Zeit seit Programmstart ab.
  if( Ergebnis_korrekt ) {
      Status_Fahrzeug = Ergebnis;  // Speichere das Ergebnis der Messung.
      letzte_Messung = Millisekunden;  // Speichere den aktuellen Zeitpunkt.
  } 
  else if(Ergebnis != PLUS_UNDEFINIERT ){
    //Serial.print("speichere Status_Fahrzeug : ");
    //Serial.println(Ergebnis);
    Status_Fahrzeug = Ergebnis;  // Speichere das Ergebnis der Messung.
  }

  // Falls innerhalb einer festgelegten Zeit der Status des E-Autos nicht ermittelt werden konnte, beende den Ladevorgang:
  if( Millisekunden > letzte_Messung + CP_TIMEOUT ) {
    Fehler( F_CP_TIMEOUT );
    //Serial.println("Timeout");
    letzte_Messung = Millisekunden;  // Variable setzen, als wäre eine korrekte Messung erfolgt, damit anschließend weitere Fehler erkannt werden können.
  }
  //Serial.print("Betriebsphase: "); 
  //Serial.println(Betriebsphase);
  if( Betriebsphase == 0 ) {  // Betriebsphase 0: Überprüfung, ob noch ein Fahrzeug vom letzten Ladevorgang angeschlossen ist, welches weitergeladen werden kann.
     //Serial.print("Status_Fahrzeug: ");
     //Serial.println(Status_Fahrzeug);
     switch( Status_Fahrzeug ) {
        case PLUS_12_V: // Wenn kein Fahrzeug angeschlossen ist, soll noch ein paar Sekunden gewartet werden. Dann wird in Betriebsphase 1 gewechselt:
           //Timer einbauen
           Betriebsphase_1();
        break;
        case PLUS_9_V: // Wenn ein Elektroauto angeschlossen ist, gehe in Betriebsphase 2:
        CP_Signal_starten();
        break;
        case PLUS_6_V:
         CP_Signal_starten();
        
        break;
        case PLUS_3_V:
        if( ADAPTERKOMPATIBILITAET && BELUEFTUNG ) CP_Signal_starten();
        else Betriebsphase_1();
        break;
        case MINUS_12_V:
        if(Pulsweite==0) Betriebsphase_1();
        else Fehler( F_CP_UNDEFINIERT );
        break;
      }
  } 
  else if( Betriebsphase == 1 ) {  // Betriebsphase 1: Auswahl der Ladestromstärke durch den Benutzer.
    
    if(fSOC>=setSOC){
         print_Line13("                 ");
         digitalWrite( O_WAKEUP, HIGH );  // Schalte CP Signal zum Auto an.  
      } 
    // Prüfe anhand des Status des Fahrzeugs, ob ein Ladevorgang gestartet werden kann:
    byte Auto_angeschlossen = 0;  // 0 -> kein Auto angeschlossen, 1 -> Auto angeschlossen, Ladung möglich, 2 -> Auto angeschlossen, aber keine Ladung möglich.
    switch( Status_Fahrzeug ) {
      case PLUS_9_V:
      Auto_angeschlossen = 1;
      break;
      case PLUS_6_V:
      Auto_angeschlossen = 2;
      break;
      case PLUS_3_V:
      if( ADAPTERKOMPATIBILITAET && BELUEFTUNG ) Auto_angeschlossen = 1;
      else Auto_angeschlossen = 2;
      break;
    }
     if( Auto_angeschlossen == 1|| Auto_angeschlossen == 2 ) CP_Signal_starten(); // Wenn ein Fahrzeug angeschlossen ist, gehe in Betriebsphase 2.
      else if ( Auto_angeschlossen == 0 ){
        delay(50);
      }
  }    
  else if( Betriebsphase == 2 ) {  // Betriebsphase 2: Warte auf Bereitschaft des E-Autos.
    Progress=0; 
    switch( Status_Fahrzeug ) {
      case PLUS_12_V:  // Wenn das Auto nicht mehr angeschlossen ist, gehe zurück zur Ladestrom-Einstellung.
      Betriebsphase_1();
      break;
      case PLUS_9_V:
      break;
      case PLUS_6_V:
      Ladevorgang_starten();
      break;
      case PLUS_3_V:
      if( BELUEFTUNG ) Ladevorgang_starten();
      else {  // Wenn keine Belüftung erlaubt ist, gehe zurück zur Ladestrom-Einstellung und lasse die Auto-LED dreimal kurz blinken:
        Betriebsphase_1();
      }
      break;
    }
  }
  else if( Betriebsphase == 3 ) {  // Betriebsphase 3: Ladevorgang.
    struct tm * timeinfo;
    int year, month, day, hour, minutes, seconds;
    String sTime;
    
    switch( Status_Fahrzeug ) {
      case PLUS_12_V: // Elektroauto ist plötzlich nicht mehr angeschlossen.
      Fehler( F_NOTABBRUCH );
      Stop_Load= time(nullptr);
      break;
      case PLUS_9_V:  // Elektroauto hat fertig geladen oder Benutzer hat Ladevorgang beendet.
      digitalWrite( O_LADUNG, LOW );  // Schalte die Stromzufuhr zum E-Auto ab.
      Stop_Load= time(nullptr);
      timeinfo = localtime(&Stop_Load); 
      year = timeinfo->tm_year+1900;
      month = timeinfo->tm_mon+1;   
      day = timeinfo->tm_mday;    
      hour = timeinfo->tm_hour;
      minutes = timeinfo->tm_min;
      seconds = timeinfo->tm_sec;
      sTime=String(hour)+":"+String(minutes)+":"+String(seconds)+" - "+String(day)+"."+String(month)+".";
      print_Line13("stop:   "+ sTime);
      print_Line12("car load done...  ");
      Betriebsphase = 4;  // Gehe in Betriebsphase 4.
      break;
      case PLUS_6_V:  // Elektroauto lädt! aber Stromzufuhr soll aus geschaltet werden weil zuwenig Strom vorhanden oder Modus geäander werden soll.
      if(fSOC<setSOC && Progress>2.5){
         print_Line13("load pausing...  ");
         digitalWrite( O_WAKEUP, LOW );  // Schalte CP Signal zum Auto ab.  
      }else{
         if (Progress/10000>100) Progress=0; 
         else Progress+=0.01; 
         
         print_Line12("car loading....  ");
         digitalWrite( O_LADUNG, HIGH );  // Schalte die Stromzufuhr zum E-Auto an.  
      }
      break;
      case PLUS_3_V:  // Wenn das Elektroauto eine Belüftung anfordert und diese nicht erlaubt ist, beende den Ladevorgang:
      if( !BELUEFTUNG ) Fehler( F_BELUEFTUNG );
      break;
    }
  } 
  else if( Betriebsphase == 4 ) {  // Betriebsphase 4: Fertig.
    
    switch( Status_Fahrzeug ) {
      case PLUS_12_V:  // Wenn das Elektroauto nicht mehr angeschlossen ist, gehe zurück zur Ladestrom-Einstellung:
      Betriebsphase_1();
      break;
      case PLUS_6_V:
      Ladevorgang_starten();
      break;
      case PLUS_3_V:
      if( BELUEFTUNG ) Ladevorgang_starten();
      else Fehler( F_BELUEFTUNG );
      break;
    }
  }
  else {  // Betriebsphase 5+: Fehler.
      if( Millisekunden < gespeicherte_Zeit ) {
        //Serial.print("Fehler ....");
        //Serial.println(Fehlercode);
        print_Line13("letzter Fehler: " + String(Fehlercode));
      }
      else {
       Fehlercode = 0;
       mydisp.setFont(10);
       mydisp.setColor(191);
       mydisp.setPrintPos(0,13,0);
       mydisp.print("                ");
       mydisp.print("  "); 
       
       Betriebsphase_0();  // Gehe in Betriebsphase 0, um zu prüfen, ob alle Bedingungen erfüllt sind.
      } 
  }
  
  ///////////////////
  // do data logging
  ///////////////////
  
  if (millis()>=ulNextMeas_ms) 
  {
    ulNextMeas_ms = millis()+ulMeasDelta_ms;
    
    pulTime[ulMeasCount%ulNoMeasValues] = time(nullptr);
    fSOC=getFEMSData("EssSoc");
    pfSOC[ulMeasCount%ulNoMeasValues]=fSOC; 
    fPower=getFEMSData("ProductionActivePower");
    pfSolar[ulMeasCount%ulNoMeasValues]=fPower;
    FreeHEAP(); // check memory leak
   // UDP write
    d2d_say_boring_life();
    ulMeasCount++;
  }
   if (millis()>=ulNextPrint_ms) 
  {
    ulNextPrint_ms = millis()+ulMeasDelta_ms;
    struct tm * timeinfo;
    time_t helper = time(nullptr);
    timeinfo = localtime(&helper); 
    int year = timeinfo->tm_year+1900;
    int month = timeinfo->tm_mon+1;   
    int day = timeinfo->tm_mday;    
    int hour = timeinfo->tm_hour;
    int minutes = timeinfo->tm_min;
    int seconds = timeinfo->tm_sec;
    String sTime=String(hour)+":"+String(minutes)+":"+String(seconds);
    mydisp.setFont(10);
    mydisp.setColor(191);
    mydisp.setPrintPos(17,0,0);
    mydisp.print(sTime);
    mydisp.print(" ");
    
    if (pfSOC[ulMeasCount%ulNoMeasValues-1]!=pfSOC[ulMeasCount%ulNoMeasValues-2]){
      show_batterie(fSOC/10,25,15,24);
      mydisp.setFont(10);
      mydisp.setColor(191);
      mydisp.setPrintPos(6,2,0);
      mydisp.print("SOC ");
      mydisp.print(fSOC);
      mydisp.print("% ");
    }
    if(pfSolar[ulMeasCount%ulNoMeasValues-1]!=pfSolar[ulMeasCount%ulNoMeasValues-2]){ 
      byte color;
      if(fPower<3000)color=224;
      else color=24;
      show_batterie(fPower/1000,25,32,color);
      mydisp.setFont(10);
      mydisp.setColor(191);
      mydisp.setPrintPos(6,4,0);
      mydisp.print("Solar ");
      mydisp.print(float(fPower/1000));
      mydisp.print("kW ");
    }
    show_batterie(STROM[Stromstaerke]*235/1000,-5,49,244);
    mydisp.setFont(10);
    mydisp.setColor(191);
    mydisp.setPrintPos(6,6,0);
    mydisp.print("Auto ");
    mydisp.print(float(STROM[Stromstaerke]*235/1000));
    mydisp.print("kW ");
    
    //show_batterie(0,0,66,244);
    //mydisp.setFont(10);
    //mydisp.setColor(191);
    //mydisp.setPrintPos(6,8,0);
    //mydisp.print("L2 ");
    //mydisp.print(2.1);
    //mydisp.print("kW ");

    //show_batterie(0,0,83,22);
    //mydisp.setFont(10);
    //mydisp.setColor(191);
    //mydisp.setPrintPos(6,10,0);
    //mydisp.print("L3 ");
    //mydisp.print(0);
    //mydisp.print("kW ");
    
    // Wenn kein Fahrzeug angeschlossen ist und die Initialisierung abgeschlossen, entriegele die Ladedose:
    if( Status_Fahrzeug == PLUS_12_V && Betriebsphase > 0 ){
       print_Line12("kein Auto angeschlossen  ");
    }
  }
  //////////////////////////////
  // check if WLAN is connected
  //////////////////////////////
  if (WiFi.status() != WL_CONNECTED)
  {
    //Serial.print("WLAN ist nicht da!");
    Fehler (F_WIFI);
    delay(1000);
    WiFi.begin();
    delay(1000);
   return;
  }
  
  ///////////////////////////////////
  // Check if a client has connected
  ///////////////////////////////////
  WiFiClient client = server.available();
  if (!client) 
  {
    return;
  }
  
  // Wait until the client sends some data
  //Serial.println("new client");
  unsigned long ultimeout = millis()+150;
  while(!client.available() && (millis()<ultimeout) )
  {
    delay(1);
  }
  if(millis()>ultimeout) 
  { 
    Serial.println("client connection time-out!");
    return; 
  }
  
  /////////////////////////////////////
  // Read the first line of the request
  /////////////////////////////////////
  String sRequest = client.readStringUntil('\r');
  //Serial.println(sRequest);
  String sBody = client.readStringUntil('\n');
  
  // stop client, if request is empty
  if(sRequest=="")
  {
    //Serial.println("empty request! - stopping client");
    client.stop();
    return;
  }
  
  // get path; end of path is either space or ?
  // Syntax is e.g. GET /?show=1234 HTTP/1.1
  String sPath="",sParam="", sCmd="";
  String sGetstart="GET ";
  int iStart,iEndSpace,iEndQuest;
  iStart = sRequest.indexOf(sGetstart);
  if (iStart>=0)
  {
    iStart+=+sGetstart.length();
    iEndSpace = sRequest.indexOf(" ",iStart);
    iEndQuest = sRequest.indexOf("?",iStart);
    
    // are there parameters?
    if(iEndSpace>0)
    {
      if(iEndQuest>0)
      {
        // there are parameters
        sPath  = sRequest.substring(iStart,iEndQuest);
        sParam = sRequest.substring(iEndQuest,iEndSpace);
      }
      else
      {
        // NO parameters
        sPath  = sRequest.substring(iStart,iEndSpace);
      }
    }
  }
  /////////////////////////////////////////////////////////////////
  // output command to serial so that we can read it on the Arduino
  /////////////////////////////////////////////////////////////////
  if(sParam.length()>0)
  {
    int iEqu=sParam.indexOf("=");
    if(iEqu>=0)
    {
      sCmd = sParam.substring(iEqu+1,sParam.length());
      
      // output to serial, which is connceted to the Arduino
      if (sCmd=="R6A")        Stromstaerke=0;
      else if(sCmd=="R10A")   Stromstaerke=1;
      else if (sCmd=="R15A")  Stromstaerke=2;
      else                    Stromstaerke=0;
      Serial.println(sCmd);
    }
  }  
  //Serial.print ("sPath: ");
  //Serial.println(sPath);
  String ipaddress = WiFi.localIP().toString(); 
  String sResponse,sRes2,sHeader, sHtmlHead;
  
  /////////////////////////////
  // format the html page for /
  /////////////////////////////
  if (sPath.substring(0,12)=="/ajax_inputs")
  {
     String sParam1="&L1=";
     String sParam2="&L2=";
     String sNocache="&noc";     
     String sValue1 = sRequest.substring(sRequest.indexOf(sParam1)+4,sRequest.indexOf(sParam2));
     String sValue2 = sRequest.substring(sRequest.indexOf(sParam2)+4,sRequest.indexOf(sNocache));
     SensorName = sValue1;                               // setzten Name des Sensors aus config.html  
     setSOC=sValue2.toInt();
     WiFi.setHostname((char*) SensorName.c_str());
     //Serial.print("Sensor Name:");
     //Serial.println(SensorName);
     
     sPath="/";
     //Serial.println(sPath);  
  }
  
  if(sPath=="/") 
  {
    unsigned long z=ulMeasCount%ulNoMeasValues-1;
    ulReqcount++;
    sHtmlHead = F("<html><head>");
    sHtmlHead +=F("<meta http-equiv=\"refresh\" content=\"");
    sHtmlHead +=ulMeasDelta_ms/1000;
    sHtmlHead +=F(";URL=/grafik\">");
    sHtmlHead +=F("<title>Wallbox@");
    sHtmlHead +=SensorName;
    sHtmlHead += F("</title>");
    sHtmlHead += F("<link rel=\"icon\" href=\"data:,\">");
    sHtmlHead += F("<style> .button {background-color: #4CAF50; border: none; color: white; padding: 15px 30px; text-align: center; text-decoration: none; display: inline-block; font-size: 18px; margin: 4px 2px; cursor: pointer; }");
    sHtmlHead += F(".button2 {background-color: #008CBA;}");
    sHtmlHead += F(".button3 {background-color: #f44336;}");
    sHtmlHead += F("</style>");
    sHtmlHead += F("<script type=\"text/javascript\" src=\"https://www.gstatic.com/charts/loader.js\"></script>\n");
    sHtmlHead += F("<script type=\"text/javascript\">google.charts.load('current', {'packages':['gauge']});google.charts.setOnLoadCallback(drawChart);");
    sHtmlHead += F("function drawChart() {");
    sHtmlHead += F("var data3=google.visualization.arrayToDataTable([");
    sHtmlHead += F("['Label', 'Value'");
    sHtmlHead += F("],['SoC (%)',");
    sHtmlHead += pfSOC[z];
    sHtmlHead += F("]]);var data4=google.visualization.arrayToDataTable([");
    sHtmlHead += F("['Label', 'Value'");
    sHtmlHead += F("],['Solar kW',");
    sHtmlHead += float(pfSolar[z])/1000;
    sHtmlHead += F("]");
    sHtmlHead += F("]);");
    
    sHtmlHead += F("var options3={max:100,width:400,height:120,redFrom:0,redTo:25,yellowFrom:25,yellowTo:75,greenFrom:75,greenTo:100,minorTicks:10,majorTicks:['0','25','50','75','100']};");
    sHtmlHead += F("var options4={max:10,width:400,height:120,redFrom:0,redTo:2,yellowFrom:2,yellowTo:5,greenFrom:5,greenTo:10,minorTicks:1,majorTicks:['0','2','4','6','8','10']};");
    sHtmlHead += F("var chart3=new google.visualization.Gauge(document.getElementById('chart_div3'));");
    sHtmlHead += F("var chart4=new google.visualization.Gauge(document.getElementById('chart_div4'));");
    sHtmlHead += F("chart3.draw(data3,options3);");
    sHtmlHead += F("chart4.draw(data4,options4);}");
    sHtmlHead += F("</script>\n</head>\n");
    sResponse = F("<body>\n<font color=\"#000000\"><body bgcolor=\"#d0d0f0\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1.0,user-scalable=yes\">");
    sResponse += F("<h1>Wallbox@");
    sResponse +=SensorName;
    sResponse += F("</h1>");
    sResponse += F("WLAN: ");
    sResponse += Router_SSID;
    sResponse += F("<br />");
    sResponse += F("StartupTime: ");
    sResponse += ctime(&Startup_time);
    sResponse += F("<br />");
    sResponse += F("Zeit: ");
    struct tm * timeinfo;
    timeinfo = localtime(&cur_time); 
    int year = timeinfo->tm_year+1900;
    int month = timeinfo->tm_mon+1;   
    int day = timeinfo->tm_mday;    
    int hour = timeinfo->tm_hour;
    int minutes = timeinfo->tm_min;
    int seconds = timeinfo->tm_sec;
    sResponse += String(hour)+":"+String(minutes)+"."+String(seconds);
    sResponse += F("&nbsp; &nbsp;");
    sResponse += String(day)+"."+String(month)+"."+String(year);
    sResponse += F("<br/>");
    sResponse += F("Status: ");
    sResponse += Stromstaerke;
    sResponse += F(" : ");
    sResponse += Betriebsphase;
    sResponse += F(" : ");
    sResponse += Pulsweite;
    sResponse += F(" ->");
    sResponse += sLine12;
    sResponse += F("<br>");
    sResponse += sLine13;
    sResponse += F("<br/>Ladestart: ");
    sResponse += ctime(&Start_Load);
    sResponse += F("<br/>Ladestop: ");
    sResponse += ctime(&Stop_Load);
    sResponse += F("</font><br>");
    sResponse +=F("<div id=\"chart_div3\" style=\"float:left\"></div>");
    sResponse +=F("<div id=\"chart_div4\" ></div>");

    //10%=6A; 16%=9,6A; 25%=15A; 30%=18A; 40%=24A; 50%=30A; 60%=36A;
    sRes2 += F("<p>");
    sRes2 += F("<a href=\"?pin=R6A\" class=\"button ");
    if (Stromstaerke==0) sRes2+=F("button3");
    else sRes2+=F("button2");
    sRes2 += F("\">6A</a>&nbsp;<a href=\"?pin=R10A\" class=\"button ");
    if (Stromstaerke==1) sRes2+=F("button3");
    else sRes2+=F("button2");
    sRes2 += F("\">10A</a>&nbsp;");
    sRes2 += F("<a href=\"?pin=R15A\" class=\"button ");
    if (Stromstaerke==2) sRes2+=F("button3");
    else sRes2+=F("button2");
    sRes2 += F("\">15A</a></p>");

    
    sRes2 += F("<p style=\"float:left;\"><FONT SIZE=+2><a href=\"/grafik\" class=\"button\"> Grafik</a><a href=\"/tabelle\" class=\"button\"> Tabelle</a><a href=\"/\" class=\"button\">Reload</a><a href=\"/config\" class=\"button button3\">Config</a>");
    sRes2 += F("<FONT SIZE=-2>");
    sRes2 += F("<br />");
    sRes2 += F("Aufrufz&auml;hler="); 
    sRes2 += ulReqcount;
    sRes2 += F(" - Verbindungsz&auml;hler="); 
    sRes2 += ulReconncount;
    sRes2 += F(" - Memory="); 
    sRes2 += freeheap;
    sRes2 += F("<br/> &#169; SchuschiLab&#10057; by Jan Schuster"); 
    sRes2 += F("</p></body></html>");
  
    sHeader  = F("HTTP/1.1 200 OK\r\n");
    sHeader += F("Content-Length: ");
    sHeader += sHtmlHead.length()+sResponse.length()+sRes2.length();
    sHeader += F("\r\n");
    sHeader += F("Content-Type: text/html\r\n");
    sHeader += F("Connection: close\r\n");
    sHeader += F("\r\n");
  }
  else if(sPath=="/tabelle")
  ////////////////////////////////////
  // format the html page for /tabelle
  ////////////////////////////////////
  {
    ulReqcount++;
    unsigned long ulSizeList = MakeTable(&client,false); // get size of table first
    
    sResponse  = F("<html>\n<head>\n<title>Wallbox@");
    sResponse  +=SensorName;
    sResponse  += F("</title></head><body>");
    sResponse += F("<font color=\"#000000\"><body bgcolor=\"#d0d0f0\">");
    sResponse += F("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=yes\">");
    sResponse += F("<h1>Wallbox@");
    sResponse  +=SensorName;
    sResponse += F("</h1>");
    sResponse += F("<FONT SIZE=+1>");
    sResponse += F("<a href=\"/\">Startseite</a><BR><BR>Letzte Messungen im Abstand von ");
    sResponse += ulMeasDelta_ms;
    sResponse += F("ms<BR>");
    // here the big table will follow later - but let us prepare the end first
      
    // part 2 of response - after the big table
    sRes2 = F("</body></html>");
    sRes2 += F("<p>Spannungsverlauf<br><a href=\"/\" class=\"button\">Startpage</a></p>");
    sHeader  = F("HTTP/1.1 200 OK\r\n");
    sHeader += F("Content-Length: ");
    sHeader += sResponse.length()+sRes2.length()+ulSizeList;
    sHeader += F("\r\n");
    sHeader += F("Content-Type: text/html\r\n");
    sHeader += F("Connection: close\r\n");
    sHeader += F("\r\n");
  }
  else if(sPath=="/grafik")
  ///////////////////////////////////
  // format the html page for /grafik
  ///////////////////////////////////
  {
    ulReqcount++;
    unsigned long ulSizeList = MakeList(&client,false); // get size of list first

    sResponse  = F("<html><head>");
    sResponse +=F("<meta http-equiv=\"refresh\" content=\"");
    sResponse +=ulMeasDelta_ms/1000;
    sResponse +=F(";URL=/grafik\">");
    sResponse +=F("<title>Wallbox@");
    sResponse += SensorName;
    sResponse += F("</title>\n");
    sResponse += F("<style> .button {background-color: #4CAF50; border: none; color: white; padding: 15px 30px; text-align: center; text-decoration: none; display: inline-block; font-size: 18px; margin: 4px 2px; cursor: pointer; }");
    sResponse += F(".button2 {background-color: #008CBA;}");
    sResponse += F(".button3 {background-color: #f44336;}");
    sResponse += F("</style>");
    sResponse += F("<style>#P{width:100%;background-color:#ddd;}#B{width:");
    sResponse += Progress/10000;
    sResponse += F("%;height:30px;background-color:#4CAF50;text-align:center;line-height:30px;color:white;}</style>");
    sResponse +=F("<script type=\"text/javascript\" src=\"https://www.gstatic.com/charts/loader.js\"></script>\n");
    sResponse += F("<script type=\"text/javascript\"> google.charts.load('current', {'packages':['corechart']}); google.charts.setOnLoadCallback(drawChart);\n"); 
    sResponse += F("function drawChart() {var data = google.visualization.arrayToDataTable([\n['Zeit / MEZ', 'SoC %', 'Solar W'],\n");
    // here the big list will follow later - but let us prepare the end first
      
    // part 2 of response - after the big list
    sRes2  = F("]);\nvar options = {title: 'Verlauf',curveType:'function', legend:{position: 'bottom'}, series:{0:{targetAxisIndex:0},1:{targetAxisIndex:1}},vAxes:{0:{title: 'SoC in %'},1:{title:'Solar Power in W'}}};");
    sRes2 += F("var chart = new google.visualization.LineChart(document.getElementById('curve_chart'));chart.draw(data, options);}\n</script>\n</head>\n");
    sRes2 += F("<body>\n<font color=\"#000000\"><body bgcolor=\"#d0d0f0\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=yes\">");
    sRes2 += F("<h1>WallBox@");
    sRes2 +=SensorName;
    sRes2 += F("</h1>");
    sRes2 += F("Status: ");
    sRes2 += Stromstaerke;
    sRes2 += F(" : ");
    sRes2 += Betriebsphase;
    sRes2 += F(" : ");
    sRes2 += Pulsweite;
    sRes2 += F(" ->");
    sRes2 += sLine12;
    sRes2 += F("<br>Status: ");
    sRes2 += sLine13;
    sRes2 += F("<br>Ladestart: ");
    sRes2 += ctime(&Start_Load);
    sRes2 += F("<br>Ladestop: ");
    sRes2 += ctime(&Stop_Load);
    sRes2 += F("<br>Fortschritt:");
    sRes2 += F("<div id=\"P\"><div id=\"B\">");
    sRes2 += Progress/10000;
    sRes2 += F("%</div></div>");

    //10%=6A; 16%=9,6A; 25%=15A; 30%=18A; 40%=24A; 50%=30A; 60%=36A;
    // Change Color bei sCmd
    sRes2 += F("<p>");
    sRes2 += F("<a href=\"?pin=R6A\" class=\"button ");
    if (Stromstaerke==0) sRes2+=F("button3");
    else sRes2+=F("button2");
    sRes2 += F("\">6A</a>&nbsp;<a href=\"?pin=R10A\" class=\"button ");
    if (Stromstaerke==1) sRes2+=F("button3");
    else sRes2+=F("button2");
    sRes2 += F("\">10A</a>&nbsp;");
    sRes2 += F("<a href=\"?pin=R15A\" class=\"button ");
    if (Stromstaerke==2) sRes2+=F("button3");
    else sRes2+=F("button2");
    sRes2 += F("\">15A</a></p>");
    
    sRes2 += F("<div id=\"curve_chart\" style=\"width: 100%; height: 500px\"></div>");
    sRes2 += F("<p><a href=\"/tabelle\" class=\"button\"> Tabelle</a><a href=\"/\" class=\"button\">Home</a><a href=\"/config\" class=\"button button3\">Config</a>");
    sRes2 += F("<FONT SIZE=-2>");
    sRes2 += F("<br />");
    sRes2 += F("Aufrufz&auml;hler="); 
    sRes2 += ulReqcount;
    sRes2 += F(" - Verbindungsz&auml;hler="); 
    sRes2 += ulReconncount;
    sRes2 += F(" - Memory="); 
    sRes2 += freeheap;
    sRes2 += F("<br/> &#169; SchuschiLab&#10057; by Jan Schuster"); 
    sRes2 += F("</p></body></html>");
    
    sHeader  = F("HTTP/1.1 200 OK\r\n");
    sHeader += F("Content-Length: ");
    sHeader += sResponse.length()+sRes2.length()+ulSizeList;
    sHeader += F("\r\n");
    sHeader += F("Content-Type: text/html\r\n");
    sHeader += F("Connection: close\r\n");
    sHeader += F("\r\n");
  }
  else if(sPath=="/config")
  ///////////////////////////////////
  // format the html page for /config
  ///////////////////////////////////
  {
    ulReqcount++;
    ///////////////////////////
    // format the html response Config Page
    ///////////////////////////
    // send web page
    sResponse =F("<html><head>");
    sResponse +=F("<meta http-equiv=\"refresh\" content=\"50;URL=/\">");
    sResponse += F("\n<title>Config@");
    sResponse  +=SensorName;
    sResponse += F("</title>");
    sResponse += F("<link rel=\"icon\" href=\"data:,\">");
    sResponse += F("<style> .button {background-color: #4CAF50; border: none; color: white; padding: 15px 30px; text-align: center; text-decoration: none; display: inline-block; font-size: 18px; margin: 4px 2px; cursor: pointer; }");
    sResponse += F(".button2 {background-color: #008CBA;}");
    sResponse += F(".button3 {background-color: #f44336;}");
    //sResponse += F(".loader {border: 16px solid #f3f3f3; border-radius: 50%; border-top: 16px solid #3498db; width: 120px; height: 120px; -webkit-animation: spin 2s linear infinite; animation: spin 2s linear infinite;}");
    //sResponse += F("@-webkit-keyframes spin { 0% { -webkit-transform: rotate(0deg); } 100% { -webkit-transform: rotate(360deg); }} @keyframes spin { 0% { transform: rotate(0deg);} 100% { transform: rotate(360deg); }}");
    sResponse += F("</style>");
    sResponse += F("</head><body><font color=\"#000000\">\n<body bgcolor=\"#d0d0f0\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=yes\">");
    sResponse += F("<script>\n");
    sResponse += F("function SaveConfig(){nocache = \"&nocache=\" + Math.random() * 1000000;\n var request = new XMLHttpRequest();\n");
    //Parameter auslesen
    sResponse += F("request.onreadystatechange = function() { if (this.readyState == 4 && this.status == 200)\n {document.write(request.responseText); } };\n");
    sResponse += F("strLine1=\"&L1=\"+document.getElementById(\"txt_form\").sensor.value;\n");
    sResponse += F("strLine2=\"&L2=\"+document.getElementById(\"txt_form\").SOC.value;");
    sResponse += F("request.open(\"GET\", \"ajax_inputs\" +strLine1+strLine2+nocache,true);\n request.send(null);}</script>");
    sResponse += F("<h1>SchuschiLab Sensor ");
    sResponse += ipaddress;
    sResponse += F(" konfigurieren</h1>");
    sResponse += F("<br/>StartupTime: ");
    sResponse += ctime(&Startup_time);
    sResponse += F("<br/>");
    sResponse += F("Letzte Messung: ");
    sResponse += ctime(&cur_time);
    sResponse += F("<br/>");
    sResponse += F("<form id=\"txt_form\" name=\"frmText\">");
    sResponse += F("<br/><label>Sensorname: <input type=\"text\" name=\"sensor\" size=\"16\"maxlength=\"16\" value=\"");
    sResponse += SensorName;
    sResponse += F("\"/></label><br/>");
    sResponse += F("<label>Set SoC: <input type=\"number\" name=\"SOC\" min=\"30\" max=\"100\" step=\"1\"value=\"");
    sResponse += setSOC;
    sResponse += F("\"/>%</label><br/>");
    sResponse += F("</form>");
    
    sRes2 = F("<br/><input class=\"button button3\" type=\"submit\" value=\"Save\" onclick=\"SaveConfig()\" />");
    sRes2 += F("<a href=\"/\" class=\"button\">Home</a></p></body></html>");
    sHeader  = F("HTTP/1.1 200 OK\r\n");
    sHeader += F("Content-Length: ");
    sHeader += sResponse.length()+sRes2.length();
    sHeader += F("\r\n");
    sHeader += F("Content-Type: text/html\r\n");
    sHeader += F("Connection: close\r\n");
    sHeader += F("\r\n");
  }
  // Check the action to see if it was a GET request.
  else if (sPath=="/favicon.ico"){
       // send a standard http response header
    sHeader  = F("HTTP/1.1 200 OK\r\n");
    sHeader += F("Content-Length: ");
    sHeader += sResponse.length()+sRes2.length();
    sHeader += F("\r\n");
    sHeader += F("Content-Type: text/html\r\n");
    sHeader += F("Connection: close\r\n");
    sHeader += F("\r\n");
     
  }
  else
  ////////////////////////////
  // 404 for non-matching path
  ////////////////////////////
  {
    sResponse="<html><head><title>404 Not Found</title></head><body><h1>Not Found</h1><p>The requested URL was not found on this server.</p></body></html>";
    
    sHeader  = F("HTTP/1.1 404 Not found\r\nContent-Length: ");
    sHeader += sResponse.length();
    sHeader += F("\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n");
  }
  
  // Send the response to the client
  client.print(sHeader);
  sHeader=F("");
  if(sPath=="/") client.print(sHtmlHead);
  sHtmlHead=F(""); 
  client.print(sResponse); 
  sResponse=F("");
  //Serial.println("Page send mit sPath: ");
  //Serial.println(sPath);
  if(sPath=="/tabelle") MakeTable(&client,true);
  if(sPath=="/grafik")  MakeList(&client,true);
  client.print(sRes2);
  sRes2=F("");
  client.stop();

  //Serial.println(F("Client disonnected"));
}
void d2d_say_boring_life(){
  //StaticJsonBuffer<255> jsonBuffer;  
  //JsonObject& root = jsonBuffer.createObject();
   uint8_t sendPacket[250];
  unsigned long z = ulMeasCount%ulNoMeasValues-1;
  String t;
  t +=F("{\"sender\":\"");
  t +=deviceID;
  t +=F("\",");
  t +=F("\"devicetype\":\"Wallbox\",");
  t +=F("\"name\":\"");
  t +=SensorName;
  t +=F("\",");
  t +=F("\"time\":");
  t +=pulTime[z];
  t +=F(",");
  t +=F("\"SOC\":"); 
  t += pfSOC[z];
  t +=F(",");
  t +=F("\"p_Solar\":"); 
  t += pfSolar[z];
  t +=F("} ");
  
  t.getBytes(sendPacket, t.length()); 
  //Serial.printf("UDP packet contents: %s\n", sendPacket);
  Udp.beginPacket("192.168.1.255", localUdpPort);
  Udp.write(sendPacket, t.length());
  Udp.endPacket(); 
  Udp.stop();
}

boolean summertime()
// European Daylight Savings Time calculation by "jurs" for German Arduino Forum
// input parameters: "normal time" for year, month, day, hour and tzHours (0=UTC, 1=MEZ)
// return value: returns true during Daylight Saving Time, false otherwise
{ 
  time_t now = time(nullptr);
  struct tm * timeinfo;
  timeinfo = localtime(&now); 
  int year = timeinfo->tm_year+1900;
  int month = timeinfo->tm_mon+1;   
  int day = timeinfo->tm_mday;    
  int hour = timeinfo->tm_hour;
  //Serial.print ("Date: "); 
  //Serial.print (day); 
  //Serial.print ("."); 
  //Serial.print (month); 
  //Serial.print ("."); 
  //Serial.println (year); 
  //Serial.print ("Time: "); 
  //Serial.println (hour); 
  byte tzHours =1;
  
  static int x1,x2, lastyear; // Zur Beschleunigung des Codes ein Cache für einige statische Variablen
  static byte lasttzHours;
  int x3;
  if (month<3 || month>10) return false; // keine Sommerzeit in Jan, Feb, Nov, Dez
  if (month>3 && month<10) return true; // Sommerzeit in Apr, Mai, Jun, Jul, Aug, Sep
  // der nachfolgende Code wird nur für Monat 3 und 10 ausgeführt
  // Umstellung erfolgt auf Stunde utc_hour=1, in der Zeitzone Berlin entsprechend 2 Uhr MEZ
  // Es wird ein Cache-Speicher für die Variablen x1 und x2 verwendet, 
  // dies beschleunigt die Berechnung, wenn sich das Jahr bei Folgeaufrufen nicht ändert
  // x1 und x2 werden nur neu Berechnet, wenn sich das Jahr bei nachfolgenden Aufrufen ändert
  if (year!= lastyear || tzHours!= lasttzHours) 
  { // Umstellungsbeginn und -ende
   x1= 1 + tzHours + 24*(31 - (5 * year /4 + 4) % 7);  
   x2= 1 + tzHours + 24*(31 - (5 * year /4 + 1) % 7);
   lastyear=year;
   lasttzHours=tzHours;
  }  
  x3= hour + 24 * day;
  if (month==3 && x3>=x1 || month==10 && x3<x2) return true; else return false;
}

//////////////////////////
// Screen Ausgabe
//////////////////////////
void print_Line12(String to_print){
  if(to_print==sLine12)return;
  sLine12=to_print;
  mydisp.setFont(10);
  mydisp.setColor(191);
  mydisp.setPrintPos(0,12,0);
  mydisp.print(sLine12);
}
void print_Line13(String to_print){
  if(to_print==sLine13)return;
  sLine13=to_print;
  mydisp.setFont(10);
  mydisp.setColor(191);
  mydisp.setPrintPos(0,13,0);
  mydisp.print(to_print);
}

void homescreen(){
  
   mydisp.clearScreen();
  
  //byte rnd_Color=24; //Grün 
  //mydisp.setColor(rnd_Color);
  mydisp.setColor(191);
  mydisp.setPrintPos(0,0,0);
  //mydisp.setColor(rnd_Color);
  mydisp.print("EVSE@Home");
  //mydisp.drawStr(0, 0, "EVSE by SchuschiLab");
  
  mydisp.drawHLine(0,14,160);
  mydisp.drawHLine(30,31,135);
  mydisp.drawHLine(0,48,160);
  mydisp.drawHLine(0,65,115);
  mydisp.drawHLine(0,82,115);
  mydisp.drawHLine(0,99,160);
  //mydisp.drawVLine(65,13,128);
  
  //Serial.print("Farbcode: ");
  //Serial.println(rnd_Color);
  //mydisp.drawStr(0, 2, "Farbcode: ");
  mydisp.drawBitmap256(2, 19, 25, 26, homeimage);
  mydisp.drawBitmap256(115, 65, 45, 21, carimage);
  
  
}  
void show_batterie(float state,byte pos_x, byte pos_y, byte color)
{  
  mydisp.setColor(0);
  //Batterie Segmente
  byte w=5;
  byte x=2;
  //1. Schritt Hintergrund reset
  mydisp.drawBox(pos_x,pos_y, 70,15);

  //2. Schritt Balken malen
  mydisp.setColor(color);
  for (byte i = 0; i <= (state*1.5); i = i + 1) //this loop will draw 10 Batterie segments
  {
    if(i>0) mydisp.drawBox((pos_x+(w*i)+(x*i)),pos_y, w,15);
    //y=y+i;
  }
}
int getFEMSData(String sValue) //FEMS Rest API Client see: https://docs.fenecon.de/de/_/latest/fems/apis.html#_fems_app_cloud_websocketjson_api.
{
  if (WiFi.status() == WL_CONNECTED){ 
    StaticJsonDocument<64> doc;
    HTTPClient http;
    WiFiClient client;
    //Example URL="http://x:user@192.168.1.24:8084/rest/channel/_sum/EssSoc"; //State of Charge
    //See Documentation FEMS API: 
    //        https://docs.fenecon.de/de/_/latest/fems/apis.html#_fems_app_restjson_api_lesend
    //
    //
    String URL="http://x:user@"+setIP+":8084/rest/channel/_sum/"; //Hausverbrauch
    
    URL=URL+sValue;
    http.begin(client, URL);
    http.setAuthorization("x", "user");
    // start connection and send HTTP header
    int httpCode = http.GET();
    // httpCode will be negative on error
    if (httpCode > 0) {
      // HTTP header has been send and Server response header has been handled
      // file found at server
      if (httpCode == HTTP_CODE_OK) {
       // get lenght of document (is -1 when Server sends no Content-Length header)
       // Parse response 
       deserializeJson(doc, http.getStream());
       // Read values
       //Serial.print("Value vom RestAPI: ");
       //Serial.println(doc["value"].as<int>());
       
      }else {
      Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
      }
    }
    http.end();
    client.stop();
    return (doc["value"].as<int>());
   }
   return(0);  
}
/*
 *  ##### #   #  #   #  #  # ##### ###  ###  #   #  ##### #   #
 *  #     #   #  ##  #  # #    #    #  #   # ##  #  #     ##  #
 *  ###   #   #  # # #  ##     #    #  #   # # # #  ###   # # #
 *  #     #   #  #  ##  # #    #    #  #   # #  ##  #     #  ##
 *  #      ###   #   #  #  #   #   ###  ###  #   #  ##### #   #
 */
 // Überträgt Ausgaben über die serielle Schnittstelle, falls DEBUG aktiviert ist:
void debug( String ausgabe ) {
  if( DEBUG ) Serial.print(ausgabe);
}
void Fehler( byte fehlercode ) {
  if( Fehlercode != 0) {
    debug( "Ignoriere Fehler.\n" );  // Wenn vorher bereits ein Fehler erkannt wurde wird der Fehler ignoriert.
  }
  else {
    digitalWrite( O_LADUNG, LOW );  // Schalte die Stromzufuhr zum E-Auto ab.
    Serial.print("Fehlercode: ");
    Serial.println(fehlercode);
    setze_CP_Signal( 0 );  // Setze den CP-Pin auf -12 V.
    Fehlercode = fehlercode; // Speichere den Fehlercode, damit anschließend die Fehler-LED entsprechend blinken kann.
    Betriebsphase = 5;
    
    gespeicherte_Zeit = Millisekunden + 1000;
  }
}
void setze_CP_Signal( byte dutyCycle ) {
  Serial.print("setzte DutyCycle:");
  Serial.println(dutyCycle);
  ledcWrite(CP_Channel, dutyCycle);
  //Serial.print("CP-Signal setzten: ");
  //Serial.println(dutyCycle);
  Pulsweite = dutyCycle;
  delay(300);
}
void Betriebsphase_0() {
  
  Betriebsphase = 0;
  print_Line12("try loading vehicle..  ");
  setze_CP_Signal( 255 );  // Setze den CP-Pin auf +12 V.
  
  gespeicherte_Zeit = Millisekunden + 10000; // In den nächsten 10 Sekunden soll eine eventuell bestehende Verriegelung nicht gelöst werden.
}

void Betriebsphase_1() {
  debug( "Einstellung Ladestrom...\n" );
  //Stromstaerke = 1;
  Betriebsphase = 1;
  setze_CP_Signal( 255 );  // Setze den CP-Pin auf +12 V.
}
void CP_Signal_starten() {
  debug( "Teile E-Auto Stromstaerke mit...\n" );
  Betriebsphase = 2;
  //Ladekabel_max_gespeichert = Ladekabel_max;  // Speichere die aktuelle Ladekabel-Belastbarkeit, um bei einer Änderung den Ladevorgang abbrechen zu können.
  gespeicherte_Zeit = Millisekunden + 60000;  // Innerhalb der nächsten 60 Sekunden muss das E-Auto reagieren.
  print_Line12("car connected with  : "+String (STROM[Stromstaerke])+"A ");
  setze_CP_Signal( PULSWEITE[Stromstaerke] );  // Beginne, dem Elektrofahrzeug die erlaubte Stromstärke mitzuteilen.
}
void Ladevorgang_starten() {
      Betriebsphase = 3;
      Start_Load= time(nullptr);
      struct tm * timeinfo;
      time_t helper = Start_Load;
      timeinfo = localtime(&helper); 
      int year = timeinfo->tm_year+1900;
      int month = timeinfo->tm_mon+1;   
      int day = timeinfo->tm_mday;    
      int hour = timeinfo->tm_hour;
      int minutes = timeinfo->tm_min;
      int seconds = timeinfo->tm_sec;
      String sTime=String(hour)+":"+String(minutes)+":"+String(seconds)+" - "+String(day)+"."+String(month)+".";
      print_Line13("start: "+sTime);
      digitalWrite( O_LADUNG, HIGH );  // Schalte die Stromzufuhr zum Elektrofahrzeug ein.
}
