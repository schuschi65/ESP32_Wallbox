/*---------------------------------------------------
eine smarte Wallbox mit Webserver for ESP32 auf Basis Arduino 

by Jan Schuster - free for anyone

Komponenten: 
- Netzteil: 12Volt Input; 5V(XD-45 MH mini); -12V Max765 https://datasheets.maximintegrated.com/en/ds/MAX764-MAX766.pdf   
- Prozessor ESP32 Dev Kit
- Digole Display (I2C)
- AD Wandler ADS1115 (I2C)
- Levelshift für CP-Signal auf 5 Volt 
- TYP2 CP-Signalisierung und Messung basiert auf der Arbeit von Michi siehe:  https://www.goingelectric.de/forum/viewtopic.php?f=34&t=22216&sid=3b178d1b2e110f8db5921a5a3405d0a5
- 2xRelais (Signalrelais für CP signal Ein/Ausschaltung. Relais zum schalten des Schützes)   

Software:
- HTTP Server zur Steuerung und Visualisierung 
- Rest API FENECON Pro Hybrid 10-Serie siehe: https://docs.fenecon.de/de/_/latest/fems/apis.html#_fems_app_restjson_api_lesend
- UDP Interface zur weiteren Nutzung der Messwerte Inhouse
---------------------------------------------------*/
#define SC_W 160  //screen width in pixels
#define SC_H 128  //screen Hight in pixels
#define _Digole_Serial_I2C_           //To tell compiler compile the special communication only, 
#include <DigoleSerial.h>             //https://www.digole.com/forum.php?topicID=1
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "time.h"
#include <DNSServer.h>              //Local DNS Server used for redirecting all requests to the configuration portal
#include <WiFiManager.h>            //https://github.com/khoih-prog/ESP_WiFiManager
#include <WiFiUdp.h>
DigoleSerialDisp mydisp(&Wire,'\x29');  //I2C:Arduino UNO: SDA (data line) is on analog input pin 4, and SCL (clock line) is on analog input pin 5
#include <ADS1115_WE.h>
#include <Wire.h>
#include "Source.h"
#define ESP_getChipId()   ((uint32_t)ESP.getEfuseMac())

////////////////////////////////////////////////////////////////////////////////////////
// Globale Variablen für Configuration
////////////////////////////////////////////////////////////////////////////////////////
String deviceID;
WiFiClient espClient;
WiFiUDP Udp;
unsigned int localUdpPort = 4210;
// WLAN where we want to be client - put your SSID/password here
WiFiManager wm;
char* uissid = "MyNewDevice";
// SSID and PW for your Router
int32_t rssi;
byte numSsid = 0;
String Router_SSID;
String Router_Pass;
String SensorName="Wallbox";      // Name des Sensors
/////////////////////////////////////////////////////////////////////////
//                             FEMS Rest API (Daten aus der Hausanlage)
////////////////////////////////////////////////////////////////////////
uint8_t fSOC;                            //Aktueller State of Charge der Batterie 
float fPower;                          // Power von Solaranlage  
uint8_t setSOC=95;                       // Defaultwert SOC aus Config.html
String setIP="fems1183.schuschi";          // Defaultwerte für FEMS REST API
WiFiClient restAPI;
/////////////////////////////////////////////////////////////////////////
//                             ntp timestamp
////////////////////////////////////////////////////////////////////////
time_t Startup_time, Start_Load, cur_time, Stop_Load;              // Startup Time des Sensors, Start_load Car , Current time
unsigned long ulcurrentmillis;
unsigned long last_pub;              // Distance to MQTT Publish
unsigned long refresh_Home;          // Timer to refresh Display
unsigned long ulReconncount;         // how often did we connect to WiFi
unsigned long ulMeasCount=0;         // values already measured
unsigned long ulNoMeasValues=0;      // size of array
unsigned long ulMeasDelta_ms;        // distance to next meas time
unsigned long ulNextMeas_ms;         // next meas time
unsigned long ulNextPrint_ms;        // next display change time
unsigned long ulPrintDelta_ms =1500; // Display Aktualisierung
unsigned long ulReqcount;       // how often has a valid page been requested
float Lademenge;
float SumLademenge;
String sLine13, sLine12;

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
int16_t CP_Plus [10], CP_Minus [10];  // Arrays für die 10-fache Messung der positiven und negativen Spannung am CP-Pin
byte CP_Status[9];                    // Zählt, wie viele Messwerte von welcher Kategorie es jeweils gibt.
byte Status_Fahrzeug = 255;           // Bei erfolgreicher Messung des Fahrzeugstatus wird dieser hier gespeichert. 
                                      //  Mögliche Werte sind PLUS_12_V, PLUS_9_V, PLUS_6_V, PLUS_3_V und MINUS_12_V. 255 bedeutet: Status wurde noch nicht erkannt.
    
#define CP_TIMEOUT 1500  // gibt an, nach welcher Zeit (in Millisekunden) ein Ladevorgang abgebrochen wird, wenn der Status des E-Autos nicht ermittelt werden kann.
byte Pulsweite, letzte_Pulsweite;  // Speichert die aktuelle Pulsweite des CP-Signals. 0 bedeutet 0 %, 255 bedeutet 100 %.
byte Ladekabel_max;  // Speichert die maximal mögliche Pulsweite aufgrund der Belastbarkeit des Ladekabels. 255 bedeutet: kein Ladekabel angeschlossen.
byte Stromstaerke=0; // Wertebereich: 0 bis 4. Speichert die eingestellte Ladestromstärke. Dient als Index für PULSWEITE und O_LED_STROM.
// Folgendes Array legt fest, welche fünf verschiedenen Ladestromstärken ausgewählt werden können. 
// Die Werte können individuell angepasst werden, müssen aber in aufsteigender Reihenfolge sortiert sein.
// Es handelt sich um 8-Bit-Werte, d.h. 0 bedeutet 0 % Pulsweite, 255 bedeutet 100 % Pulsweite.
//10%=6A; 16%=9,6A; 25%=15A; 30%=18A; 40%=24A; 50%=30A; 60%=36A;
const byte PULSWEITE [5] = {   255*10/100,   255*16/100, 255*25/100 , 255*30/100,  255*40/100};    // 6A, 9,6A, 15A, 18A, 24A

// Durch die folgenden Werte werden die ADC-Messwerte dividiert, um die Spannung am CP-Pin zu erhalten.
// PWM to Spannung 1000 hz = 1 ms: Um=Uaus+((Uein−Uaus)⋅tein/(tein+taus))  Uaus=-12V Uein=12V  tein=1000*Pulsweite/255 taus=1000-tein=1000-(1000*Pulsweite/255)  
const uint16_t  FAKTOR_PLUS[5]={8,17,29,34,44};  //original {9,17,29,34,44};
const byte STROM[5] = { 6, 10 , 15 , 18 , 24 };
byte Fehlercode = 0;  // 0 bedeutet: kein Fehler. Andere mögliche Werte sind die mit F_ gekennzeichneten Konstanten (siehe oben unter DEFINITIONEN).

// setting PWM properties for ESP32
#define freq            1000
#define CP_Channel      0
#define resolution      8

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

////////////////////////////////////////////////////////////////////////////////////////
// storage for Measurements; keep some mem free; allocate remainder
int freeheap;
#define KEEP_MEM_FREE 185000
#define MEAS_SPAN_H 25
unsigned long *pulTime;         // array for time points of measurements
uint8_t *pfSOC;                     // array SOC Battery 
uint16_t *pfSolar;                  // array PowerSolar
uint16_t *pfVerbrauch;              // array Verbrauch  
// Create an instance of the server on Port 80
WiFiServer server(80);
//////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Sensor Settings
//////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////
//Analog digital Wandler ADS1115
//////////////////////////////
ADS1115_WE ads = ADS1115_WE();


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
// Helper to String
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
   // configure PWM functionalitites
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
  // start Display
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
  //AD Settings
  ads.setVoltageRange_mV(ADS1115_RANGE_2048);  //comment line/change parameter to change range 
  ads.setCompareChannels(ADS1115_COMP_0_GND);
  ads.setMeasureMode(ADS1115_CONTINUOUS);
  ads.setConvRate(ADS1115_64_SPS);
  
  uint32_t free=esp_get_free_heap_size() - KEEP_MEM_FREE;
  Serial.print("freeHeap:");
  Serial.println(free);
  ulNoMeasValues = free / (sizeof(uint8_t)+2*sizeof(uint16_t)+sizeof(unsigned long));  // SOC+PowerSolar+time 
  Serial.print("number Messuresvalues:");
  Serial.println(ulNoMeasValues);
  pulTime = new unsigned long[ulNoMeasValues];
  pfSOC = new uint8_t[ulNoMeasValues];
  pfSolar = new uint16_t[ulNoMeasValues];
  pfVerbrauch = new uint16_t[ulNoMeasValues];
  
  if (pulTime==NULL || pfSOC==NULL|| pfSolar==NULL|| pfVerbrauch==NULL)
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
  
  WiFi.hostname(SensorName);
  WiFiStart();
  
  FreeHEAP();
  freeheap = ESP.getFreeHeap();
  // Start the server
  server.begin();
  Serial.print("Server listening on Port 80: ");
  // Print the IP address
  Serial.println(WiFi.localIP());
  homescreen();
  refresh_Home=millis()+(1000*3600);  //Display refresh jede Stunde
  letzte_Messung = millis();
  digitalWrite( O_WAKEUP, HIGH );  // Schalte CP Signal zum Auto an.
  OTAStart();
}


///////////////////
// (re-)start WiFi
///////////////////
void WiFiStart()
{
  //--unsigned long startedAt = millis();
  deviceID =  uint64ToString(ESP.getEfuseMac());     // IoT thing device ID - unique device id in our project
  freeheap = ESP.getFreeHeap();
  delay(random(2000));
  ulReconncount++;
  mydisp.setColor(191);
  mydisp.drawStr(0, 0, "....Connecting");
  WiFi.mode(WIFI_STA);
  //WiFi.persistent (true);
  std::vector<const char *> menu = {"wifi","info","param","sep","restart","exit"};
  wm.setMenu(menu);
  // set dark theme
  wm.setClass("invert");
  wm.setMinimumSignalQuality(70);  // set min RSSI (percentage) to show in scans, null = 8%
  // Connect to WiFi network
  wm.setConfigPortalTimeout(120); // auto close configportal after n seconds
  bool res;
  // res = wm.autoConnect(); // auto generated AP name from chipid
  res = wm.autoConnect(uissid); // anonymous ap
  //res = wm.autoConnect(uissid, password); // password protected ap
   
  if(!res) {
    Serial.println("Failed to connect or hit timeout");
    // ESP.restart();
  } 
  else {
    //if you get here you have connected to the WiFi    
    Serial.println("connected...");
  }  
  // We can't use WiFi.SSID() in ESP32as it's only valid after connected. 
  // SSID and Password stored in ESP32 wifi_ap_record_t and wifi_config_t are also cleared in reboot
  // Have to create a new function to store in EEPROM/SPIFFS for this purpose
     //--Router_SSID = wifiManager.WiFi_SSID();
     //--Router_Pass = wifiManager.WiFi_Pass();
  //strcpy(uissid, WiFi.SSID().c_str());
  Router_SSID=WiFi.SSID();
  Serial.println (Router_SSID);
  Serial.print ("  Signalstrength: ");
  Serial.println (WiFi.RSSI());
  rssi = WiFi.RSSI();
    
  
  mydisp.setColor(191);
  mydisp.drawStr(0, 0, "WIFI Connected");
  
 ///////////////////////////////
  // connect to NTP and get time
  ///////////////////////////////
  configTime(0,0, "pool.ntp.org", "time.nist.gov");
  setenv("TZ", "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00", 1);
  Serial.println("\nWaiting for time");
  mydisp.setColor(191);
  mydisp.drawStr(0, 0, "Waiting for time");
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
    sTable = "<table style=\"width:100%\"><tr><th>Zeit / MEZ</th><th>SoC (%)</th><th>Solar W</th><th>Verbrauch W</th></tr>";
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
      sTable += "</td><td>";
      sTable += pfVerbrauch[ulIndex];
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
      sTable += ",";
      sTable += pfVerbrauch[ulIndex];
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
// !!!!!IMPORTANT: Wichtig ist das Setup des AD Wandlers! 
// Zu schnell eingestellt, wird bei niedrigem DutyCycle die Low Phase gemessen.
float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  
  ads.setCompareChannels(channel); //comment line/change parameter to change channel
  delay(10);
  voltage = ads.getResult_mV(); // alternative: getResult_mV for Millivolt
  return voltage;
}
bool CP_messen(){
  //Serial.println("Start--------------------------------");
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
    if(Pulsweite==255)     Spannung_Plus= (float)(CP_Plus[index]/133.3); //133.3 alte version!! Divisor muß mit dem Messgerät ermittelt werden
    else if (Pulsweite==0) Spannung_Plus=0;
    else                   Spannung_Plus= (float)(CP_Plus[index]/FAKTOR_PLUS[Stromstaerke]); //Divisor muß mit dem Messgerät ermittelt werden
    cp_P_durchschnitt +=Spannung_Plus;
    float Spannung_Minus = (float)CP_Minus[index]/165;   //165 alte version Divisor muß mit dem Messgerät ermittelt werden
    //Serial.print( " +" + String(Spannung_Plus, 2) + " V, -" + String(Spannung_Minus, 2) + " V," );
    
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
  
  //debug("\nAnzahl Werte pro Bereich: ");
  //for(byte i = 0; i < 10; i++) Serial.println( CP_Status[i] );
  //debug("\n");
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
  
  //zur Entwicklung nützlich: Debug wenn die Messwerte nicht korrekt:  
  //if ( Ergebnis == PLUS_UNDEFINIERT){
  // Serial.print("CP zu hoch anzahl: ");
  // Serial.println(CP_Status[ZU_HOHE_SPANNUNG]);
  // Serial.print("CP 12V anzahl: ");
  // Serial.println(CP_Status[PLUS_12_V]); 
  // Serial.print("CP 9V anzahl: ");
  // Serial.println(CP_Status[PLUS_9_V]); 
  // Serial.print("CP 6V anzahl: ");
  // Serial.println(CP_Status[PLUS_6_V]); 
  // Serial.print("CP undefined anzahl: ");
  // Serial.println(CP_Status[PLUS_UNDEFINIERT]); 
  // Serial.print("CP nicht negativ anzahl: ");
  // Serial.println( CP_Status[NICHT_NEGATIV]); 
  // Serial.print("Stromstärke: ");
  // Serial.println(Stromstaerke);
  // Serial.print("Faktor: ");
  // Serial.println(FAKTOR_PLUS[Stromstaerke]);
  // Serial.print("Pulsweite: ");
  // Serial.println(Pulsweite);
  // Serial.print("CP durchschnitt: ");
  // Serial.println(cp_P_durchschnitt/10);
  // Serial.print("vorläufiges Ergebnis(CP-Tabelle!!): ");
  // Serial.println(Ergebnis);
  // Serial.print("Realwerte CP_PLUS[x]: ");
  // for(byte i = 0; i < 10; i++) {Serial.print(CP_Plus[i]); Serial.print(" , ");}  
  // Serial.println(" "); 
  // Serial.print("Realwerte CP_MINUS[x]: ");
  // for(byte i = 0; i < 10; i++) {Serial.print(CP_Minus[i]); Serial.print(" , ");}  
  // Serial.println(" "); 
  //}

  // Kein Messwert darf höher als das Ergebnis sein:
  Ergebnis_korrekt &= ( CP_Status[ZU_HOHE_SPANNUNG] < 1 );
  Ergebnis_korrekt &= ( CP_Status[PLUS_12_V] == 0 || Ergebnis == PLUS_12_V );
  Ergebnis_korrekt &= ( CP_Status[PLUS_9_V]  == 0 || Ergebnis == PLUS_9_V || Ergebnis == PLUS_12_V || Ergebnis == PLUS_UNDEFINIERT );
  Ergebnis_korrekt &= ( CP_Status[PLUS_6_V]  == 0 || ( Ergebnis != PLUS_3_V && Ergebnis != MINUS_12_V ) );
  Ergebnis_korrekt &= ( CP_Status[PLUS_3_V]  == 0 || Ergebnis != MINUS_12_V );
  Ergebnis_korrekt &= ( CP_Status[PLUS_UNDEFINIERT] == 0 || Ergebnis != MINUS_12_V );
 
  byte Mindestanzahl_Werte_Plus = 7 ;
  Ergebnis_korrekt &= ( CP_Status[Ergebnis]      >= Mindestanzahl_Werte_Plus );
 
  if( Ergebnis == PLUS_UNDEFINIERT ) Ergebnis_korrekt=false; 
  
  return (Ergebnis_korrekt);
}

/////////////
// main look
/////////////
void loop() 
{
   ArduinoOTA.handle();
  cur_time = time(nullptr);
  //Timer reset bei überlauf
  if (millis()<ulcurrentmillis){last_pub=0; CP_Messung=0;refresh_Home=0;} 
  ulcurrentmillis = millis();

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
  // Ladekabel fest auf 85
   Ladekabel_max =  85;
  
  
  //Timer CP Messung
  if (ulcurrentmillis>=CP_Messung||Pulsweite!=letzte_Pulsweite){
      Ergebnis_korrekt=CP_messen();
      //Serial.print("Returnwerte aus CP_messen()=");
      //Serial.print(Ergebnis_korrekt);Serial.println("Messwert: ");Serial.println(Ergebnis);
      CP_Messung=ulcurrentmillis+(500);
      letzte_Pulsweite=Pulsweite; 
  }
  
  Millisekunden = millis();  // Rufe die vergangene Zeit seit Programmstart ab.
  if( Ergebnis_korrekt ) {
      Status_Fahrzeug = Ergebnis;  // Speichere das Ergebnis der Messung.
      letzte_Messung = Millisekunden;  // Speichere den aktuellen Zeitpunkt.
  } 
  else if(Ergebnis != PLUS_UNDEFINIERT ){
    Status_Fahrzeug = Ergebnis;  // Speichere das Ergebnis der Messung.
  }

  // Falls innerhalb einer festgelegten Zeit der Status des E-Autos nicht ermittelt werden konnte, beende den Ladevorgang:
  if( Millisekunden > letzte_Messung + CP_TIMEOUT ) {
    Fehler( F_CP_TIMEOUT );
    letzte_Messung = Millisekunden;  // Variable setzen, als wäre eine korrekte Messung erfolgt, damit anschließend weitere Fehler erkannt werden können.
  }
  if( Betriebsphase == 0 ) {  // Betriebsphase 0: Überprüfung, ob noch ein Fahrzeug vom letzten Ladevorgang angeschlossen ist, welches weitergeladen werden kann.
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
    SumLademenge+=Lademenge;
    Lademenge=0;
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
      case PLUS_6_V:  // Elektroauto lädt! 
         if(fSOC+5<(setSOC)){    //5% SOC Konstante Hysterese aber Stromzufuhr soll aus geschaltet werden weil zuwenig Strom vorhanden oder Modus geäander werden soll.
           print_Line13("load pausing...  ");
           digitalWrite( O_WAKEUP, LOW );  // Schalte CP Signal zum Auto ab.
           //Progress=0;  
         }else{
           if (Progress/10000>100){ Progress=0;} 
           else {Progress+=0.02;} 
           Lademenge=(float)(cur_time-Start_Load)/3600*STROM[Stromstaerke]*2*235/1000.00;  
           //print_Line12("car loading....  ");
           print_Line12("loading: "+String(Lademenge,2));
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
      String Fehlertext;
      if( Millisekunden < gespeicherte_Zeit ) {
        //Serial.print("Fehler ....");
        //Serial.println(Fehlercode);
        
        switch(Fehlercode){
          case F_TEMP_MAX:
          Fehlertext=" :Temp_max";
          break;
          case F_TEMP_MIN:
          Fehlertext=" :Temp_min";
          break;
          case F_VERRIEGELUNG:
          Fehlertext=" :Verriegelung";
          break;
          case F_PP_UNDEFINIERT:
          Fehlertext=" :PP-error";
          break;
          case F_PP_FEHLT:
          Fehlertext=" :PP-unkown";
          break;
          case F_CP_TIMEOUT:
          Fehlertext=" :CP-Timeout";
          break;   
          case F_CP_UNDEFINIERT:
          Fehlertext=" :CP-unknown";
          break;   
          case F_CP_SPANNUNG:
          Fehlertext=" :CP-range";
          break;
          case F_CP_DIODE:
          Fehlertext=" :CP-Diode";
          break;
          case F_CP_KURZSCHLUSS:
          Fehlertext=" :CP-GND";
          break;
          case F_NOTABBRUCH:
          Fehlertext=" :broken";
          break;
          case F_SCHALTER:
          Fehlertext=" :phase changed";
          break;
          case F_BELUEFTUNG:
          Fehlertext=" :Belüftung";
          break;
          case F_WIFI:
          Fehlertext=" :WIFI";
          break; 
        }
        print_Line13("letzter Fehler: " + String(Fehlercode)+Fehlertext);
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
    pfVerbrauch[ulMeasCount%ulNoMeasValues]=getFEMSData("ConsumptionActivePower");
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
      byte color;
      if(fSOC/10<setSOC)color=224;
      else color=24;
      show_batterie(fSOC/10,25,15,color);
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
      mydisp.print("solar ");
      mydisp.print(float(fPower/1000));
      mydisp.print("kW ");
    }
    if(pfVerbrauch[ulMeasCount%ulNoMeasValues-1]!=pfVerbrauch[ulMeasCount%ulNoMeasValues-2]){ 
      int z=ulMeasCount%ulNoMeasValues;
      z=z-1;
      if (z>0){
        byte color;
        if(pfVerbrauch[z]<3000)color=24;
        else color=224;
        show_batterie(float(pfVerbrauch[z])/1000,-5,49,color);
        mydisp.setFont(10);
        mydisp.setColor(191);
        mydisp.setPrintPos(0,6,0);
        mydisp.print("Verbrauch ");
        mydisp.print(float(pfVerbrauch[z])/1000);
        mydisp.print("kW ");
      }  
    }
    // todo: Ladestärke realwerte einbauen
    if (Betriebsphase == 3){
       show_batterie(float(STROM[Stromstaerke]*2*235)/1000,-5,66,244);  //Auto mit 2 Phasen (VW-Konzern)
       mydisp.setFont(10);
       mydisp.setColor(191);
       mydisp.setPrintPos(0,8,0);
       mydisp.print("Auto ");
       mydisp.print(float(STROM[Stromstaerke]*2*235)/1000);      //Auto mit 2 Phasen (VW-Konzern)
       mydisp.print(" ");
    }else{
       show_batterie(0,-5,66,244);
       mydisp.setFont(10);
       mydisp.setColor(191);
       mydisp.setPrintPos(0,8,0);
       mydisp.print("Auto ");
       mydisp.print(0);
       mydisp.print("kW ");
    }
    
    // Wenn kein Fahrzeug angeschlossen ist und die Initialisierung abgeschlossen
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
    WiFi.hostname(SensorName);
    WiFi.begin();
    delay(5000);
   //return;
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
  Serial.println("new client");
  unsigned long ultimeout = millis()+250;
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
      else if (sCmd=="R18A")  Stromstaerke=3;
      else                    Stromstaerke=0;
      Serial.println(sCmd);
    }
  }  

  String ipaddress = WiFi.localIP().toString(); 
  String sResponse,sRes2,sHeader;
  
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
     WiFi.hostname(SensorName);
     //Serial.print("Sensor Name:");
     //Serial.println(SensorName);
     
     sPath="/";
     //Serial.println(sPath);  
  }
  
  if(sPath=="/") 
  {
    unsigned long z=ulMeasCount%ulNoMeasValues-1;
    ulReqcount++;
    sResponse = F("<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">");
    sResponse +=F("<meta http-equiv=\"refresh\" content=\"");
    sResponse +=ulMeasDelta_ms/1000;
    sResponse +=F(";URL=/\">");
    sResponse +=F("<title>");
    sResponse +=SensorName;
    sResponse +=F("@");
    sResponse +=ipaddress;
    sResponse += F("</title>");
    sResponse += F("<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">");
    sResponse += F("<link rel=\"icon\" href=\"data:,\">");
    sResponse += F("<style>.button{background-color: #4CAF50;border: none; color: white; padding: 10px 15px; text-align: center; text-decoration: none; display: inline-block; font-size: 15px; margin: 4px 2px; cursor: pointer; }");
    sResponse += F(".button2 {background-color: #008CBA;} /* Blue */");
    sResponse += F(".button3 {background-color: #f44336;} /* Red */ .topnav {overflow: hidden; background-color: #50B8B4; color: white; font-size: 1rem; max-width: 50em;text-align:center;margin:.5em;padding:.5em;}");
    sResponse += F(".flex-container{display:flex;max-width: 50em;}.wrap{flex-wrap: wrap;} .flex-item{flex:auto; margin:.5em;padding:.5em;background:white;text-align:center; box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5);}</style>");
    sResponse += F("</head><body {margin: 0; background: white;}>");
    sResponse += F("<script type=\"text/javascript\" src=\"https://www.gstatic.com/charts/loader.js\"></script>\n");
    sResponse += F("<script type=\"text/javascript\"> google.charts.load('current', {'packages':['gauge']}); google.charts.setOnLoadCallback(drawChart);");
    sResponse += F("function drawChart() {");
    sResponse += F("var data=google.visualization.arrayToDataTable([");
    sResponse += F("['Label', 'Value'");
    sResponse += F("],['SoC(%)',");
    sResponse += pfSOC[z];
    sResponse += F("],['Solar',");
    sResponse += float(pfSolar[z])/1000;
    sResponse += F("],['Verbrauch',");
    sResponse += float(pfVerbrauch[z])/1000;
    sResponse += F("]]);");
    
    sResponse += F("var options={max:100,width:400,height:120,redFrom:0,redTo:25,yellowFrom:25,yellowTo:75,greenFrom:75,greenTo:100,minorTicks:10,majorTicks:['0','25','50','75','100']};");
    sResponse += F("var chart=new google.visualization.Gauge(document.getElementById('chart_div'));");
    sResponse += F("chart.draw(data,options);}");
    sResponse += F("</script>\n");
    sResponse += F("<script type=\"text/javascript\">function myhref(web){window.location.href = web;}</script>");
    sResponse += F("<font color=\"#000000\">");
    sResponse += F("<div class=\"topnav\"><h2>");
    sResponse += SensorName;
    sResponse += "@";
    sResponse += ipaddress;
    sResponse += F("</h2></div>");
    
    struct tm * timeinfo;
    timeinfo = localtime(&cur_time); 
    int year = timeinfo->tm_year+1900;
    int month = timeinfo->tm_mon+1;   
    int day = timeinfo->tm_mday;    
    int hour = timeinfo->tm_hour;
    int minutes = timeinfo->tm_min;
    int seconds = timeinfo->tm_sec;
    
    sResponse += F("<div class=\"flex-container wrap\">");
    sResponse += F("<p class=\"flex-item\" onmouseover=\"this.style.background='gray';\" onmouseout=\"this.style.background='white';\" onclick=\"myhref('/config');\">Zeit: ");
    sResponse += String(hour) + ":" + String(minutes) + ":" + String(seconds);
    //sResponse += "</p><p class=\"flex-item\">";
    sResponse += "<br>Datum:" + String(day) + "." + String(month) + "." + String(year);
    sResponse += F("<br>WLAN: ");
    sResponse += Router_SSID;
    sResponse += F("<br>Signal: ");
    if (rssi >= -50) sResponse += F("100");
    else sResponse += 2 * (rssi + 100);
    sResponse += F("% <br>DeviceID: ");
    sResponse += deviceID;
    sResponse += F("<br>StartupTime: ");
    sResponse += ctime(&Startup_time);
    sResponse += F("</p></div>");
    sResponse += F("<div class=\"flex-container wrap\"><p class=\"flex-item\">");
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
    sResponse += F("</p>");
    
    sResponse += F("<p class=\"flex-item\">Gesamtenergie<br>Auto laden: ");
    sResponse += SumLademenge;
    sResponse += F(" kWh</p>");
    
    sResponse += F("<p class=\"flex-item\">");
    sResponse += F("Ladestart: ");
    sResponse += ctime(&Start_Load);
    sResponse += F("<br>Ladezeit: ");
    if( Betriebsphase == 3 ){
      sResponse += float(cur_time-Start_Load)/60;
      sResponse += F(" Minuten");
    }
    sResponse += F("</p><p class=\"flex-item\" onmouseover=\"this.style.background='gray';\" onmouseout=\"this.style.background='white';\" onclick=\"myhref('/grafik');\">Lademenge: ");
    sResponse += Lademenge;
    sResponse += F(" kWh<br/>Autobatterie geladen: ");
    sResponse += Lademenge/17*100;
    sResponse += F(" %<br/>Reichweite: ");
    sResponse += Lademenge/25*100;
    sResponse += F(" Kilometer</p>");
    sResponse += F("</div></font>");

    
    
    sResponse +=F("<div id=\"chart_div\" ></div>");
    sRes2 += F("<p>");
    sRes2 += F("<a href=\"?pin=R6A\" class=\"button ");
    if (Stromstaerke==0) sRes2+=F("button3");
    else sRes2+=F("button2");
    sRes2 += F("\">2.8kWh</a>&nbsp;<a href=\"?pin=R10A\" class=\"button ");
    if (Stromstaerke==1) sRes2+=F("button3");
    else sRes2+=F("button2");
    sRes2 += F("\">4.6kWh</a>&nbsp;");
    sRes2 += F("<a href=\"?pin=R15A\" class=\"button ");
    if (Stromstaerke==2) sRes2+=F("button3");
    else sRes2+=F("button2");
    sRes2 += F("\">6.9kWh</a></p>");
    sRes2 += F("<FONT SIZE=-2>Aufrufz&auml;hler="); 
    sRes2 += ulReqcount;
    sRes2 += F(" - Verbindungsz&auml;hler="); 
    sRes2 += ulReconncount;
    sRes2 += F(" - Memory="); 
    sRes2 += freeheap;
    sRes2 += F("<br/> &#169; SchuschiLab&#10057; by Jan Schuster"); 
    sRes2 += F("</p></body></html>");
  
    sHeader  = F("HTTP/1.1 200 OK\r\n");
    sHeader += F("Content-Length: ");
    sHeader += sResponse.length()+sRes2.length();
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
    
    sResponse  = F("<html>\n<head>\n<title>Sensor@");
    sResponse  +=SensorName;
    sResponse  += F("</title></head><body>");
    sResponse += F("<font color=\"#000000\"><body bgcolor=\"#d0d0f0\">");
    sResponse += F("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=yes\">");
    sResponse += F("<h1>Sensor@");
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

    sResponse  = F("<!DOCTYPE html><html lang=\"en\"><head>");
    sResponse +=F("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><meta http-equiv=\"refresh\" content=\"");
    sResponse +=ulMeasDelta_ms/1000;
    sResponse +=F(";URL=/grafik\"><title>Wallbox@");
    sResponse += SensorName;
    sResponse += F("</title>\n<style> .button {background-color: #4CAF50; border: none; color: white; padding: 15px 30px; text-align: center; text-decoration: none; display: inline-block; font-size: 18px; margin: 4px 2px; cursor: pointer; }");
    sResponse += F(".button2 {background-color: #008CBA;}\n .button3 {background-color: #f44336;}");
    sResponse += F("</style>\n<style>#P{width:100%;background-color:#ddd;}#B{width:");
    sResponse += Progress/10000;
    sResponse += F("%;height:30px;background-color:#4CAF50;text-align:center;line-height:30px;color:white;}</style>");
    sResponse +=F("<script type=\"text/javascript\" src=\"https://www.gstatic.com/charts/loader.js\"></script>\n");
    sResponse += F("<script type=\"text/javascript\"> google.charts.load('current', {'packages':['corechart']}); google.charts.setOnLoadCallback(drawChart);\n"); 
    sResponse += F("function drawChart() {var data = google.visualization.arrayToDataTable([\n['Zeit / MEZ', 'Batterie', 'Erzeugung', 'Verbrauch'],\n");
    // here the big list will follow later - but let us prepare the end first
      
    // part 2 of response - after the big list
    sRes2  = F("]);\nvar options = {title: 'Verlauf',curveType:'function', legend:{position: 'bottom'}, series:{0:{targetAxisIndex:0},1:{targetAxisIndex:1},2:{targetAxisIndex:1}},vAxes:{0:{title: 'SoC (%)'},1:{title:'Power (W)'}}};");
    sRes2 += F("var chart = new google.visualization.LineChart(document.getElementById('curve_chart'));chart.draw(data, options);}\n</script>\n</head>\n");
    sRes2 += F("<body>\n<font color=\"#000000\"><body bgcolor=\"#F0F8FF\">");
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
    sRes2 += F("<br/>Ladezeit: ");
    if( Betriebsphase == 3 ){
      sRes2 += float(cur_time-Start_Load)/60;
    }else{
      sRes2 += float(Stop_Load-Start_Load)/60;
    } 
    sRes2 += F(" Minuten<br>Fortschritt:");
    sRes2 += F("<div id=\"P\"><div id=\"B\">");
    sRes2 += Progress/10000;
    sRes2 += F("%</div></div>");

    //10%=6A; 16%=9,6A; 25%=15A; 30%=18A; 40%=24A; 50%=30A; 60%=36A;
    // Change Color bei sCmd
    sRes2 += F("<p>");
    sRes2 += F("<a href=\"?pin=R6A\" class=\"button ");
    if (Stromstaerke==0) sRes2+=F("button3");
    else sRes2+=F("button2");
    sRes2 += F("\">2.8kWh</a>&nbsp;<a href=\"?pin=R10A\" class=\"button ");
    if (Stromstaerke==1) sRes2+=F("button3");
    else sRes2+=F("button2");
    sRes2 += F("\">4.6kWh</a>&nbsp;");
    sRes2 += F("<a href=\"?pin=R15A\" class=\"button ");
    if (Stromstaerke==2) sRes2+=F("button3");
    else sRes2+=F("button2");
    sRes2 += F("\">6.9kWh</a></p>");
    
    sRes2 += F("<div id=\"curve_chart\" style=\"width: 100%; height: 500px\"></div>");
    sRes2 += F("<p><a href=\"/tabelle\" class=\"button\"> Tabelle</a><a href=\"/\" class=\"button\">Home</a><a href=\"/config\" class=\"button button3\">Config</a>");
    sRes2 += F("<FONT SIZE=-2><br />Aufrufz&auml;hler="); 
    sRes2 += ulReqcount;
    sRes2 += F(" - Verbindungsz&auml;hler="); 
    sRes2 += ulReconncount;
    sRes2 += F(" - Memory="); 
    sRes2 += freeheap;
    sRes2 += F("<br/> &#169; SchuschiLab&#10057; by Jan Schuster</p></body></html>");
    
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
    sResponse += F("request.onreadystatechange = function() { if (this.readyState == 4 && this.status == 200)\n {window.location='/';} };\n");
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
  while (client.available()) {
    // but first, let client finish its request
    // that's diplomatic compliance to protocols
    // (and otherwise some clients may complain, like curl)
    // (that is an example, prefer using a proper webserver library)
    client.read();
  }
  // Send the response to the client
  client.println(sHeader + sResponse + "\n");
  delay(1);
  //client.print(sResponse2);
  //Serial.println(sHeader);
  //Serial.println("Client disonnected");
  
  if(sPath=="/tabelle") MakeTable(&client,true);
  if(sPath=="/grafik")  MakeList(&client,true);
  client.print(sRes2+ "\n");
  sRes2=F("");
  client.stop();

  //Serial.println(F("Client disonnected"));
}
void d2d_say_boring_life(){
  StaticJsonDocument<250> doc;

  unsigned long z = ulMeasCount%ulNoMeasValues-1;
  doc["Sender"] = deviceID;
  doc["DeviceType"] ="Wallbox";
  doc["DeviceName"] =SensorName;
  doc["SoC"] = pfSOC[z];
  doc["Erzeugung"]= pfSolar[z];
  doc["Verbrauch"]= pfVerbrauch[z];
  doc["Time"] =pulTime[z];

  //Serial.printf("UDP packet contents: %s\n", sendPacket);
  Udp.beginPacket("192.168.1.255", localUdpPort);
  serializeJson(doc, Udp);
  Udp.println();
  Udp.endPacket(); 
  Udp.stop();
}
//////////////////////////
// Display Ausgaben
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
  mydisp.setColor(191);
  mydisp.setPrintPos(0,0,0);
  
  mydisp.print("EVSE@Home");
  mydisp.drawHLine(0,14,160);
  mydisp.drawHLine(30,31,135);
  mydisp.drawHLine(0,48,160);
  mydisp.drawHLine(0,65,115);
  mydisp.drawHLine(0,82,115);
  mydisp.drawHLine(0,99,160);
 
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
  mydisp.drawBitmap256(115, 65, 45, 21, carimage);
}
int getFEMSData(String sValue) //FEMS Rest API Client see: https://docs.fenecon.de/de/_/latest/fems/apis.html#_fems_app_cloud_websocketjson_api.
{
  if (WiFi.status() == WL_CONNECTED){ 
    StaticJsonDocument<255> doc;
    HTTPClient http;
    
    //Example URL="http://x:user@192.168.1.24:8084/rest/channel/_sum/EssSoc"; //State of Charge
    //See Documentation FEMS API: 
    //        https://docs.fenecon.de/de/_/latest/fems/apis.html#_fems_app_restjson_api_lesend
    String URL="http://x:user@"+setIP+":8084/rest/channel/_sum/"; //Base Url 
    
    URL=URL+sValue;
    //Serial.println(URL);
    http.begin(restAPI, URL);
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
       //String sStream;
       //sStream =http.getString();
       //Serial.println (sStream); 
       deserializeJson(doc, http.getStream());
       
      }else {
      //Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
      }
    }
    http.end();
    restAPI.stop();
    //Serial.println(doc);
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
    //Serial.print("Fehlercode: ");
    //Serial.println(fehlercode);
    setze_CP_Signal( 0 );  // Setze den CP-Pin auf -12 V.
    Fehlercode = fehlercode; // Speichere den Fehlercode, damit anschließend die Fehler-LED entsprechend blinken kann.
    Betriebsphase = 5;
    
    gespeicherte_Zeit = Millisekunden + 1000;
  }
}
void setze_CP_Signal( byte dutyCycle ) {
  //Serial.print("setzte DutyCycle:");
  //Serial.println(dutyCycle);
  ledcWrite(CP_Channel, dutyCycle);
  Pulsweite = dutyCycle;
  delay(500);
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
void OTAStart() {
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(SensorName.c_str());

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}
