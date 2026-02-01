# ros_nxt
moveit+nxt

# Generalidades (A completar)
## Adquisición y Control
### Pines Header- Pin NXT
| PIN header Kicad | Column 2 | Column 3 |
|------------------|----------|----------|
|     1            | Cell 2   | Cell 3   |
|     2            | Cell 5   | Cell 6   |
|     3            | Cell 8   | Cell 9   |

### Pines salida servomotor 
| PIN del cable original | Color cable|         Funcion             | Señal/Polaridad | forma de señal |
|------------------------|------------------------------------------|-----------------|----------------|
|     1                  |  Blanco    |  Motor Power Supply         |     DC +/- 9V   |       DC       |
|     2                  |  Negro     |  Motor Power Supply         |     DC +/- 9V   |       DC       |
|     3                  |  Rojo      |  Power Detector de rotacion |     GND         |       DC       | 
|     4                  |  Verde     |  Power Detector de rotacion |     +4.3V a 5V  |       DC       |
|     4                  |  Amarillo     |  0/5 V contra pin 3 o -5/0 V contra pin 4|     +4.3V a 5V  |       rectangulo       |
|     4                  |  Azul         |  0/5 V contra pin 3 o -5/0 V contra pin 4|     +4.3V a 5V  |       rectangulo       |

![image](https://github.com/user-attachments/assets/f4c6ca01-31e6-4fa3-9935-ac6e417892ad)
