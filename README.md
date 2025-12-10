# CAN_ESP32

# Como hacer build
1. Instalamos esp-idf (para poder visualizarlo lo instalamos en vscode)
2. Clonamos o descargamos el repositorio
3. Movemos la carpeta `.espressif` a components (suele estar en `C:\Users\<usuario>\.espressif`)
4. Buideamos y compilamos el proyecto

## Continguts d'aquest repo actualment: 
En aquest repo, ara per ara, trobem codi del màster. A master_init s'han fet també canvis però l'integració d'està fent aquí dins. El màster segueix la lògica explicada als esquemes. Aquí va una petita explicació del que hi ha a les funcions concretes. 

### main
-> void setup_hardware_master(): en aquesta funció s'inicialitza el ESP sobre el que s'està carregant el màster, així com els pins de GPIO per poder fer a posteriori la integració amb la part d'interrupcions per GPIO. 

-> void app_main(): inicialitzem memòria no volàtil, configurem els pins, executem la lògica del màster, bucle del main per l'execució del màster de forma permanent. 

### CANopen_LSS_Master
-> void CO_ESP32_Master_Run(void): el que fem aquí és crear una tasca recurrent que generi totes les funcions del màster. 

-> void CO_mainTask(void *pxParam): executa les tasques de la inicialització dels nodes amb el gestor del CAN, LSS, i inicialitza totes les funcionalitats del CAN i del CANopen, així com les diverses tipologies de missatges i de mòduls. 

-> uint32_t calcularNodeID(CO_LSS_adress_t identity): calcula el Node ID de cadascun dels nodes connectats al bus a través de l'adreça LSS. L'adreça LSS als nodes està configurada amb els paràmetres de vendorID, productID, revisionINumber, SerialNumber. A partir d'aquests paràmetres  es genera una adreça LSS única als slaves. A partir d'aquesta adreça generem el nodeID. 



