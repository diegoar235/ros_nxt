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
    - Ejecutar: **git clone https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git**, clona el proyecto micro_ros_stm32cubemx_utils 
    - Esto descarga el builder preparado para ROS 2 Humble. "docker pull microros/micro_ros_static_library_builder:humble" (para humble, importante!)
    - Limpiá cualquier intento previo (para no mezclar salidas) -> Esto no o hice, pero habria que ver si es necesario, al menos no lo hice
        - rm -rf micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros
        - rm -rf micro_ros_stm32cubemx_utils/microros_static_library_ide/build
        - rm -rf micro_ros_stm32cubemx_utils/microros_static_library_ide/install
        - rm -rf micro_ros_stm32cubemx_utils/microros_static_library_ide/log
- Executar docker:
  - dmesg | tail -n 30
  - ls -l /dev/ttyACM0
  - docker run -it --rm --net=host --privileged -v /dev:/dev
        microros/micro-ros-agent:humble
        serial --dev /dev/ttyACM0 -v6
- con el paso anterior, se crea:
    ls -la micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros
    Y adentro deberías ver:
    libmicroros.a
    include/
- Configurar CubeIDE para que compile con micro-ROS:
    1-Include path (headers): Project → Properties → C/C++ Build → Settings → Tool Settings → MCU GCC Compiler → Includes **/home/diego/Documentos/Comm2/microros_f411_cdc/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include**  lo que finalmente se tiene que agregar es la carpeta include
    2-Linker: ruta de librería (-L): **/home/diego/Documentos/Comm2/microros_f411_cdc/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros**
    3-Libraries (-l): **microros**
    4-Agregar los “extra sources” que micro-ROS necesita: desde micro_ros_stm32cubemx_utils/extra_sources/ agregar 
        microros_time.c
        microros_allocators.c
        custom_memory_manager.c



## Configuracion de timmers:
- Configuración de PWM (TIM 5)
    - Pines usados en ambas STM: TIM5_CH1 = PA0, TIM5_CH2 = PA1, TIM5_CH3 = PA2.
    - Prescaler=0
    - Period=4199
    - Con clock (TIM5 ≈ 84 MHz), eso te da: 20kHz
- Encoders (TIM2/TIM3/TIM4)
    - Pines usados TIM2: CH1: PA5, CH2: PB3
    - Pines usados TIM3: CH1: PA6, CH2: PA7
    - Pines usados TIM4: CH1: PB6, CH2: PB7
    - Prescaler=0
    - EncoderMode=TI12 (x4)
    - IC1Filter=IC2Filter=4
    - HAL_TIM_Encoder_Start(..., TIM_CHANNEL_ALL)

  

