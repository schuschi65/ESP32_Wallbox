# ESP_Wallbox
 Smarte Wallbox: ESP32 mit Arduino
 
 Wallbox Webserver for ESP32 
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

