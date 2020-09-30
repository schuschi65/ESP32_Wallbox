# ESP32 Wallbox
Eine smarte Wallbox mit dem ESP32 und Integration zur Haus Solaranlage auf Basis Arduino
 
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


Das Elektroauto wird mittels Typ2 Stecker wie gewohnt angeschlossen. 
Wird nun genug Sonnenenergie erzeugt, der Hausspeicher auf einen über die Weboberfläche einstellbaren Wert geladen ist,
dann wird das Auto mit dem eigenen Strom geladen. 
D.h. das Relais für den Schütz zum Fahrzeug welches am ESP32 (GPIO19) hängt wird geschalten.
Die Ladestärke ist über die Weboberfläche einstellbar (6A, 10A, 15A...). Gegebenfalls auch mehr.  
Die Ladestärke wird über den DutyCycle am ESP32 (GPIO23) gesteuert. 

Sinkt der Ladezustand der Hausbatterie weil die Solaranlage nicht genügend Strom liefert, wird das CP Signal über den ESP32 (GPIO18)
getrennt. Dadurch wird die Aufladung des Fahrzeuges unterbrochen. Die Trennung ist erforderlich damit das Auto nicht "einschläft".  

Das PWM Signal der CP Leitung wird ständigg über den AD Wandler überwacht und beeinflusst die nachfolgende Steuerung.      

In der Weboberfläche werden die aktuellen Leistungsdaten (SoC, sowie Erzeugung der Solaranlage) aus der eigenen Solaranlage angezeigt. 
Die Werte werden aus der FEMS von Fenecon per Rest ausgelesen https://www1.fenecon.de/fems/    
Desweiteren wird der aktuelle Status des Fahrzeuges in der Weboberfläche angezeigt.


Webserver example: 

https://github.com/schuschi65/ESP_Wallbox/blob/main/screenshot1.JPG