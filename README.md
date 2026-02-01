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
|     3                  |  Rojo      |  Power Detector de rotacion              |     GND         |       DC       | 
|     4                  |  Verde     |  Power Detector de rotacion              |     +4.3V a 5V  |       DC       |
|     4                  |  Amarillo  |  0/5 V contra pin 3 o -5/0 V contra pin 4|     +4.3V a 5V  |       rectangulo       |
|     4                  |  Azul      |  0/5 V contra pin 3 o -5/0 V contra pin 4|     +4.3V a 5V  |       rectangulo       |

# SMT 32
## Configuracion de micro-ros:
- Pasos para configurar la instalacion de microros
    - Ir a la carpeta donde está el archivo .ioc: en el caso que tenia es "cd /home/diego/Documentos/Comm2/microros_f411_cdc"
    - Esto descarga el builder preparado para ROS 2 Humble. "docker pull microros/micro_ros_static_library_builder:humble" (para humble, importante!)
    - Limpiá cualquier intento previo (para no mezclar salidas) -> Esto no o hice, pero habria que ver si es necesario, al menos no lo hice
        - rm -rf micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros
        - rm -rf micro_ros_stm32cubemx_utils/microros_static_library_ide/build
        - rm -rf micro_ros_stm32cubemx_utils/microros_static_library_ide/install
        - rm -rf micro_ros_stm32cubemx_utils/microros_static_library_ide/log
  -4 docker run --rm \
    -v "$PWD":/project \
    -e MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide \
    microros/micro_ros_static_library_builder:humble
  -5 con el paso anterior, se crea:
    ls -la micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros
    Y adentro deberías ver:
    libmicroros.a
    include/
  -6 Configurar CubeIDE para que compile con micro-ROS:
    1-Include path (headers): Project → Properties → C/C++ Build → Settings → Tool Settings → MCU GCC Compiler → Includes (/home/diego/Documentos/Comm2/microros_f411_cdc/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include)  lo que finalmente se tiene que agregar es la carpeta include
    2-Linker: ruta de librería (-L) y nombre (-l): /home/diego/Documentos/Comm2/microros_f411_cdc/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros
    3-Libraries (-l): microros
    4-Agregar los “extra sources” que micro-ROS necesita: desde micro_ros_stm32cubemx_utils/extra_sources/ agregar 
        microros_time.c
        microros_allocators.c
        custom_memory_manager.c

## Configuracion de timmers:
- 

