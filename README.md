# ros_nxt
moveit+nxt

# Generalidades (A completar)
## Adquisición y Control
### Pines Header- Pin NXT
|    PIN header NXT  | PIN idc 2x3  |
|--------------------|--------------|
|     1 - Blanco     | Cell 2       |
|     2 - Negro      | Cell 5       |
|     3 - Rojo       | Cell 8       |
|     4 - Verde      | Cell 8       |
|     5 - Amarillo   | Cell 8       |
|     6 - Azul       | Cell 8       |

### Pines conector NXT para servomotor-
| PIN del cable original | Color cable|         Funcion             | Señal/Polaridad | forma de señal |
|------------------------|------------|-----------------------------|-----------------|----------------|
|     1                  |  Blanco    |  Motor Power Supply         |     DC +/- 9V   |       DC       |
|     2                  |  Negro     |  Motor Power Supply         |     DC +/- 9V   |       DC       |
|     3                  |  Rojo      |  Power Detector de rotacion |     GND         |       DC       | 
|     4                  |  Verde     |  Power Detector de rotacion |     +4.3V a 5V  |       DC       |
|     4                  |  Amarillo  |  0/5 V contra pin 3 o -5/0 V contra pin 4|     +4.3V a 5V  |       rectangulo       |
|     4                  |  Azul      |  0/5 V contra pin 3 o -5/0 V contra pin 4|     +4.3V a 5V  |       rectangulo       |


