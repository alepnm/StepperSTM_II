
BENDRAS:



INIT steite ne darom:
* skaitom draiverio statusa
* skaitom analoginius iejimus
* skaitom skaitmeninius iejimus
* atnaujinam sistemine aplinka
* atnaujinam modbus registrus




******** INIT ********
* inicializuojam hw
* atstatom eeproma
* inicializuojam sistemine aplinka
* inicializuojam modbus registrus
* inicializuojam modbus steka


Pereinam i STOP;
******** STOP ********
* keiciam variklio konfiguracija

Pereinam i NORMAL, FAULT, SCROLL, TEST, CONFIG;
******** NORMAL ********
* keiciam greiti, kripti, stabdom varikli
* tikrinam HOLO daviklio aktyvuma

Pereinam i STOP, FAULT;
******** SCROLL ********
* tikrinam HOLO daviklio aktyvuma


Pereinam i STOP, FAULT;
******** TEST ********


Pereinam i STOP;
