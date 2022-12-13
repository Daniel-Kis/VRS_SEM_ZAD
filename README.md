# VRS - Charakteristika Osvetlenia

Cieľ projektu je pomocou viacsegmentového svetelného senzora vykresliť charakteristiku okolitého osvetlenia. Senzor bude otáčaní pomocou servo motora.
Pomocou STM32F303K8 mikrokontrolera sa budú spracovať dáta zo senzora, ktoré následne budú poslaté prostredníctvom USART do PC. V Matlabe bude vytvorená aplikácia, ktorá vizualizuje charakteristiku okolitého osvetlenia.

# VRS - Characteristics of ambient lighting

The goal of this project is to visualize the characteristics of ambient lighting using a light sensor that is rotated with a servo motor.
The STM32F303K microcontroller will be used to to process the raw data from the light sensor, which will then be sent to the PC via USART. This data will then be visualized in a Matlab app.

![alt text](https://github.com/Daniel-Kis/VRS_SEM_ZAD/blob/main/Images/VRS_FINAL_FLOWCHART.drawio.png?raw=true)

(Flowchart of the communication between hardware)
