STATIE METEO INTELIGENTA CU MONITORIZARE MULTI-PARAMETRU

Autor: Nicolaescu Alex
Platforma: ATmega328P (Arduino Uno/Nano)
Mediu de dezvoltare: PlatformIO / VS Code
Namespace Wiki: pm/prj2026/atoader/anicolaescu2602

--- DESCRIERE PROIECT ---
Proiectul consta intr-o statie meteorologica avansata capabila sa masoare
si sa proceseze temperatura, umiditatea, presiunea atmosferica si
intensitatea luminoasa. Sistemul include un meniu interactiv salvat in
EEPROM, feedback vizual prin LED RGB (tranzitie PWM) si alerte sonore
dinamice.

--- STRUCTURA ARHIVEI ---
.
├── src/
│   └── main.cpp 
├── include/              
├── lib/                   
├── schematics/  
│    └── anicolaescu2602_circuit.png
│    └── anicolaescu2602_schema_bloc          
├── platformio.ini         
└── README.md            

ChangeLog

v1.2 - 2026-05-12: Adaugat meniu setari EEPROM, filtru presiune si Mute.
v1.1 - 2026-05-08: Integrat BMP280, calcul altitudine si unitati masura.
v1.0 - 2026-05-02: Citire AHT20, driver LCD I2C si control LED RGB.
v0.5 - 2026-04-25: Prototip comunicatie I2C si teste PWM Buzzer.

--- FUNCTIONALITATI CHEIE ---

Monitorizare: 3 pagini de date (Temp/Hum, Presiune, Altitudine/Lux).

Setari: Meniu pentru configurarea pragurilor de alerta (apasa lung D2).

Memorie: Salvarea permanenta a preferintelor in EEPROM.

Smart Mute: Dezactivare sonora automata pe timp de noapte (<10 lx).

Filtrare: Medie mobila pentru stabilizarea citirilor barometrice.

Acest proiect a fost realizat in scop didactic pentru laboratorul de
Proiectarea cu Microprocesoare (PM).