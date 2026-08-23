# Instructions

+ Before to start Docker, enable the STMs, by means of:asegurarse de tener 
    + lsusb (to be sure the STMs are connected)
    + ls -l /dev/bus/usb/003/005 (according to the previuos command)
    + sudo chmod 666  (this allows privilegies)
+ by-id
    + (B) usb-STMicroelectronics_STM32_Virtual_ComPort_388D39643132-if00 ()
    + (A) usb-STMicroelectronics_STM32_Virtual_ComPort_389639893132-if00
## To Execute Docker:

1) xhost +SI:localuser:root
2) Source: https://github.com/NVIDIA/nvidia-container-toolkit/issues/48
    
    ./gui-docker -v $HOME/Documentos/ROS2/NXT:/root/ros_ws --rm -it \
    --device=/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_388D39643132-if00 \
    --device=/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_389639893132-if00 \
    --net=host \
    --privileged \
    --runtime=nvidia --gpus all \
    --device=/dev/nvidia-uvm \
    --device=/dev/nvidia-uvm-tools \
    --device=/dev/nvidia-modeset \
    --device=/dev/nvidiactl \
    --device=/dev/nvidia0 \
    ros2_moveit_humble:latest /bin/bash

3) Inside docker "export LIBGL_ALWAYS_SOFTWARE=1"

4) Then continue with: "source install/setup.bash"

## Note:
+ If it requires a different container, replace the line "docker.io/moveit/moveit2:humble-source" with the corresponding container
+     --device=/dev/ttyACM0:/dev/ttyACM0 \  --device=/dev/ttyACM1:/dev/ttyACM1 \



=====================================================================================
Para todos los contenedores:
  docker stop $(docker ps -q)


=====================================================================================

 comando prueba
 ros2 topic pub -r 10 /stm32B/cmd_deg std_msgs/msg/Int32MultiArray "{data: [0, 0, 0]}"
   
 
    # Lego/config/ros2_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100   # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_position_controller:
      type: forward_command_controller/ForwardCommandController

joint_state_broadcaster:
  ros__parameters: {}
  
arm_position_controller:
  ros__parameters:
    type: forward_command_controller/ForwardCommandController
    joints: [joint1, joint2, joint3, joint4]
    interface_name: position
    
    
    # Instalar paquetes si aún faltan
sudo apt update
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers


===================================================================================== 
Center pose
Api key for NVIDIA docker container
dnZvN3YyN3FiMWF1aTdyaW1iMnZnbzhzdHY6NWQzMWFjZmItZDYyZi00MzMxLTg0OGYtMzQ0MWE2NTFhYmZj

Para ejecutar centerpose


cd /home/diego/Documentos/ROS2/RPOSE
./src/isaac_ros_common/scripts/run_dev.sh -d /home/diego/Documentos/ROS2/RPOSE

Dentro :

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch isaac_ros_centerpose isaac_ros_centerpose_tensor_rt.launch.py \
  model_file_path:=isaac_ros_assets/models/centerpose_shoe/1/model.onnx \
  object_name:=shoe


=====================================================================================
Datos de la pinza

abre:
    gripper:
    - 0.0
    - 0.0
cierra
    gripper:
    - 0.015905697966786102
    - 0.015915364174800925                   

=====================================================================================
MOVER EN GAZEBO    
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'],
  points: [
    {
      positions: [0, 0, 0, 0, 0, 0],
      time_from_start: {sec: 2, nanosec: 0}
    }
  ]
}"
=====================================================================================
MOVER REAL
ros2 action send_goal /BRAZO_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'],
    points: [
      {
        positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        time_from_start: {sec: 4, nanosec: 0}
      }
    ]
  }
}"



temp

FROM ros2_moveit_humble:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt update && apt install -y \
    git \
    python3-pip \
    python3-pykdl \
    python3-colcon-common-extensions \
    ros-humble-kdl-parser \
    ros-humble-urdfdom-py \
    ros-humble-pinocchio \
    ros-humble-foxglove-bridge \
    liborocos-kdl-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt/kdl_parser_py_ws/src && \
    cd /opt/kdl_parser_py_ws/src && \
    git clone https://github.com/ros/kdl_parser_py.git && \
    cd /opt/kdl_parser_py_ws && \
    . /opt/ros/humble/setup.sh && \
    colcon build --merge-install

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /opt/kdl_parser_py_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/ros_ws

========================================================================================
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
ros2 run lego play_sequence_full.py --ros-args -p archivo_yaml:=poses2.yaml

========================================================================================
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "uros_transport_cdc.h"
#include "motor_pwm.h"

#include <stdbool.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/bool.h>

#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32.h>
#include "encoders.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;

static rcl_publisher_t pub_enc;
static std_msgs__msg__Int32MultiArray msg_enc;

static rcl_subscription_t sub_cmd_deg;
static std_msgs__msg__Int32MultiArray msg_cmd_deg;
static int32_t cmd_deg_buf[3];

static rcl_subscription_t sub_gripper;
static std_msgs__msg__Bool msg_gripper;

static rcl_publisher_t pub_z_eff;
static std_msgs__msg__Float32 msg_z_eff;

// último comando recibido
static volatile int32_t  cmd_cdeg[3] = {0, 0, 0};
static volatile bool     gripper_closed = false;
static volatile uint32_t cmd_last_ms = 0;
static volatile uint32_t grip_last_ms = 0;

static float z_eff_cdeg_dbg = 0.0f;

// buffer fijo para publicar encoders
static int32_t enc_buf[3];

// =====================
// Seguridad / watchdog
// =====================
#define CMD_TIMEOUT_MS        500u

// Límite de PWM en modo seguro.
// Ajustar según fuerza necesaria para sostener el eje.
#define SAFE_PWM_LIMIT        80.0f

// Límites de comando en centésimas de grado.
// Ajustar a tus límites reales.
#define CMD_MIN_CDEG         -36000
#define CMD_MAX_CDEG          36000

typedef enum {
  CTRL_WAITING_CMD = 0,
  CTRL_RUNNING    = 1,
  CTRL_SAFE_HOLD  = 2
} control_mode_t;

static volatile control_mode_t ctrl_mode = CTRL_WAITING_CMD;
static volatile bool safety_latched = false;
static volatile uint32_t safety_latched_ms = 0;

// Referencias virtuales internas.
// En RUNNING se mueven hacia cmd_cdeg.
// En SAFE_HOLD quedan congeladas.
static float spY_ref = 0.0f;
static float spZ_ref = 0.0f;
static float spX_ref = 0.0f;

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

static rclc_executor_t executor;

/* USER CODE BEGIN PFP */
static bool cmd_msg_valid(const std_msgs__msg__Int32MultiArray *m);
static bool cmd_timeout_expired(void);
static void reset_pi_all(void);
static void enter_comm_lost_stop(void);
static void capture_current_virtual_refs(void);
static void enter_safe_hold(void);
static float clamp_pwm_safe(float u);
static void motors_stop_all(void);
static void emergency_outputs_off(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// Encoder en eje motor: 720 counts por vuelta
static const int32_t CPR_MOTOR = 720;

// ---- helpers ----
static inline float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// cdeg -> counts motor
static inline float cdeg_to_counts_motor(float cdeg)
{
  return (cdeg * (float)CPR_MOTOR) / 36000.0f;
}

static inline float counts_to_cdeg_motor(float counts)
{
  return (counts * 36000.0f) / (float)CPR_MOTOR;
}

// ---- DWT ----
static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// ---- PI ----
typedef struct {
  float kp, ki, dt;
  float i;
  float out_min, out_max;
} pi_t;

static float pi_update(pi_t *p, float e)
{
  float i_new = p->i + e * p->dt;
  float u_unsat = p->kp * e + p->ki * i_new;
  float u = clampf(u_unsat, p->out_min, p->out_max);

  // anti-windup simple
  if (u == u_unsat) {
    p->i = i_new;
  } else {
    if ((u == p->out_max && e < 0) || (u == p->out_min && e > 0)) {
      p->i = i_new;
    }
  }

  return u;
}

// referencia de velocidad por motor, counts/s
// [0]=M4, [1]=M5, [2]=M6
static float v_ref_motor[3] = {0.0f, 0.0f, 0.0f};

// PI velocidad por motor
static pi_t pi_v[3] = {
  //{ .kp=0.03f, .ki=0.20f, .dt=0.001f, .i=0.0f, .out_min=-127.0f, .out_max=127.0f }, // M4
  { .kp=0.15f, .ki=5.00f, .dt=0.001f, .i=0.0f, .out_min=-127.0f, .out_max=127.0f },
  //{ .kp=0.03f, .ki=0.20f, .dt=0.001f, .i=0.0f, .out_min=-127.0f, .out_max=127.0f }, // M5
  { .kp=0.15f, .ki=5.00f, .dt=0.001f, .i=0.0f, .out_min=-127.0f, .out_max=127.0f },
  //{ .kp=0.03f, .ki=0.20f, .dt=0.001f, .i=0.0f, .out_min=-127.0f, .out_max=127.0f }  // M6
  { .kp=0.15f, .ki=5.00f, .dt=0.001f, .i=0.0f, .out_min=-127.0f, .out_max=127.0f },
};

// ---- micro-ROS ----
static void wait_for_agent(void)
{
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    HAL_Delay(200);
  }
}

// =====================
// Seguridad
// =====================
static bool cmd_msg_valid(const std_msgs__msg__Int32MultiArray *m)
{
  if (m == NULL) return false;
  if (m->data.data == NULL) return false;
  if (m->data.size < 3) return false;

  for (int i = 0; i < 3; i++)
  {
    if (m->data.data[i] < CMD_MIN_CDEG || m->data.data[i] > CMD_MAX_CDEG)
    {
      return false;
    }
  }

  return true;
}

static bool cmd_timeout_expired(void)
{
  // Si todavía nunca recibí comando, no enclavo por timeout.
  // El estado WAITING_CMD ya mantiene motores quietos.
  if (cmd_last_ms == 0u)
  {
    return false;
  }

  return ((uint32_t)(HAL_GetTick() - cmd_last_ms) > CMD_TIMEOUT_MS);
}

static void reset_pi_all(void)
{
  for (int i = 0; i < 3; i++)
  {
    pi_v[i].i = 0.0f;
    v_ref_motor[i] = 0.0f;
  }
}

static void capture_current_virtual_refs(void)
{
  int32_t p4 = Encoders_GetPos(0);
  int32_t p5 = Encoders_GetPos(1);
  int32_t p6 = Encoders_GetPos(2);

  float qY = 0.5f * ((float)p4 + (float)p5);
  float qZ = 0.5f * ((float)p4 - (float)p5);
  float qX = (float)p6;

  spY_ref = qY;
  spZ_ref = qZ;
  spX_ref = qX;
}

static void enter_safe_hold(void)
{
  if (ctrl_mode != CTRL_SAFE_HOLD)
  {
    ctrl_mode = CTRL_SAFE_HOLD;
    safety_latched = true;
    safety_latched_ms = HAL_GetTick();

    // Congelar referencia en la posición real actual.
    capture_current_virtual_refs();

    // Evita tirones por integral acumulada.
    reset_pi_all();
  }
}
static void enter_comm_lost_stop(void)
{
  // Perdida de comunicacion: no enclavar falla.
  // Apaga motores y queda esperando un nuevo comando valido.
  ctrl_mode = CTRL_WAITING_CMD;
  safety_latched = false;
  cmd_last_ms = 0u;

  reset_pi_all();
  motors_stop_all();
}
static float clamp_pwm_safe(float u)
{
  if (u >  SAFE_PWM_LIMIT) return  SAFE_PWM_LIMIT;
  if (u < -SAFE_PWM_LIMIT) return -SAFE_PWM_LIMIT;
  return u;
}

static void emergency_outputs_off(void)
{
  // Baja PWM si TIM5 ya fue inicializado.
  if (htim5.Instance != NULL)
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
  }

  // Baja direcciones.
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);
}

// ---- callbacks ----
static void cmd_deg_cb(const void * msgin)
{
  const std_msgs__msg__Int32MultiArray * m =
      (const std_msgs__msg__Int32MultiArray *)msgin;

  if (!cmd_msg_valid(m))
  {
    enter_safe_hold();
    return;
  }

  // Comunicación viva
  cmd_last_ms = HAL_GetTick();

  // Si ya entró en falla, no aceptar comandos automáticamente.
  // Para salir de SAFE_HOLD, por ahora resetear la STM32
  // o luego agregar un tópico reset_fault.
  if (safety_latched)
  {
    return;
  }

  // Primer comando válido: sincronizo referencias internas
  // con la posición real actual, para evitar saltos.
  if (ctrl_mode == CTRL_WAITING_CMD)
  {
    capture_current_virtual_refs();
    reset_pi_all();
  }

  cmd_cdeg[0] = m->data.data[0]; // Y
  cmd_cdeg[1] = m->data.data[1]; // Z
  cmd_cdeg[2] = m->data.data[2]; // X

  ctrl_mode = CTRL_RUNNING;
}

static void gripper_cb(const void * msgin)
{
  const std_msgs__msg__Bool * m = (const std_msgs__msg__Bool *)msgin;
  gripper_closed = m->data;
  grip_last_ms = HAL_GetTick();
}

// ---- Actuadores ----
// TIM5_CH1 (PA0), DIR: PA4/PA8
void Servo_M4(int8_t pwm)
{
  uint16_t ccr = (pwm == 0) ? 0 : pwm_to_ccr(pwm);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, ccr);

  if (pwm > 0) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  } else if (pwm < 0) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  }
}

// TIM5_CH2 (PA1), DIR: PB4/PB5
void Servo_M5(int8_t pwm)
{
  uint16_t ccr = (pwm == 0) ? 0 : pwm_to_ccr(pwm);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, ccr);

  if (pwm > 0) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  } else if (pwm < 0) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  }
}

// TIM5_CH3 (PA2), DIR: PC13/PC14
void Servo_M6(int8_t pwm)
{
  uint16_t ccr = (pwm == 0) ? 0 : pwm_to_ccr(pwm);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, ccr);

  if (pwm > 0) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
  } else if (pwm < 0) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
  }
}

static void motors_stop_all(void)
{
  Servo_M4(0);
  Servo_M5(0);
  Servo_M6(0);
}

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  dwt_init();

  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  HAL_Delay(1500);

  uros_setup_transport_cdc();
  wait_for_agent();

  // --- rclc init ---
  allocator = rcl_get_default_allocator();

  rcl_ret_t rc;
  rc = rclc_support_init(&support, 0, NULL, &allocator);
  if (rc != RCL_RET_OK) Error_Handler();

  rc = rclc_node_init_default(&node, "stm32B", "", &support);
  if (rc != RCL_RET_OK) Error_Handler();

  // Subs: /stm32B/cmd_deg y /stm32B/gripper_cmd
  rclc_subscription_init_default(
    &sub_cmd_deg,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/stm32B/cmd_deg"
  );

  rclc_subscription_init_default(
    &sub_gripper,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/stm32B/gripper_cmd"
  );

  rclc_executor_init(&executor, &support.context, 2, &allocator);

  rclc_executor_add_subscription(
    &executor,
    &sub_cmd_deg,
    &msg_cmd_deg,
    &cmd_deg_cb,
    ON_NEW_DATA
  );

  rclc_executor_add_subscription(
    &executor,
    &sub_gripper,
    &msg_gripper,
    &gripper_cb,
    ON_NEW_DATA
  );

  // msg_cmd_deg sin malloc
  msg_cmd_deg.data.data = cmd_deg_buf;
  msg_cmd_deg.data.size = 3;
  msg_cmd_deg.data.capacity = 3;
  msg_cmd_deg.layout.dim.data = NULL;
  msg_cmd_deg.layout.dim.size = 0;
  msg_cmd_deg.layout.dim.capacity = 0;
  msg_cmd_deg.layout.data_offset = 0;

  // Publisher /stm32B/encoders
  rc = rclc_publisher_init_default(&pub_enc,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),"/stm32B/encoders");
  if (rc != RCL_RET_OK) Error_Handler();

  rc = rclc_publisher_init_default(&pub_z_eff,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),"/stm32B/z_eff_cdeg");
  if (rc != RCL_RET_OK) Error_Handler();

  msg_enc.data.data = enc_buf;
  msg_enc.data.size = 3;
  msg_enc.data.capacity = 3;
  msg_enc.layout.dim.data = NULL;
  msg_enc.layout.dim.size = 0;
  msg_enc.layout.dim.capacity = 0;
  msg_enc.layout.data_offset = 0;

  // HW init
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  Encoders_Reset();

  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

  motors_stop_all();

  // Tiempos
  uint32_t t_last_100 = HAL_GetTick();

  const uint32_t CPU_HZ = 84000000u;
  const uint32_t DT1MS_CYC = CPU_HZ / 1000u;
  uint32_t last_cyc = DWT->CYCCNT;

  while (1)
  {
    // Procesar RX micro-ROS
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

    // ==========================================================
    // Lazo rápido 1 kHz: PI velocidad -> PWM
    // ==========================================================
    uint32_t now = DWT->CYCCNT;
    if ((uint32_t)(now - last_cyc) >= DT1MS_CYC)
    {
      last_cyc += DT1MS_CYC;

      Encoders_Step_1kHz();

      // Watchdog de comunicación.
      // Solo se evalúa si ya estaba corriendo.
      if (ctrl_mode == CTRL_RUNNING && cmd_timeout_expired())
      {
        enter_comm_lost_stop();
      }

      if (ctrl_mode == CTRL_WAITING_CMD)
      {
        reset_pi_all();
        motors_stop_all();
      }
      else
      {
        // M4=idx0, M5=idx1, M6=idx2
        int16_t d4 = Encoders_GetDelta(0);
        int16_t d5 = Encoders_GetDelta(1);
        int16_t d6 = Encoders_GetDelta(2);

        float v4 = (float)d4 / 0.001f;
        float v5 = (float)d5 / 0.001f;
        float v6 = (float)d6 / 0.001f;

        float vr4 = clampf(v_ref_motor[0], -800.0f, 800.0f);
        float vr5 = clampf(v_ref_motor[1], -800.0f, 800.0f);
        float vr6 = clampf(v_ref_motor[2], -800.0f, 800.0f);

        float u4 = pi_update(&pi_v[0], (vr4 - v4));
        float u5 = pi_update(&pi_v[1], (vr5 - v5));
        float u6 = pi_update(&pi_v[2], (vr6 - v6));

        // En modo seguro, limitar esfuerzo.
        if (ctrl_mode == CTRL_SAFE_HOLD)
        {
          u4 = clamp_pwm_safe(u4);
          u5 = clamp_pwm_safe(u5);
          u6 = clamp_pwm_safe(u6);
        }

        Servo_M4((int8_t)u4);
        Servo_M5((int8_t)u5);
        Servo_M6((int8_t)u6);
      }
    }

    // ==========================================================
    // Lazo lento 100 Hz: diferencial Y/Z/X -> v_ref_motor
    // ==========================================================
    if ((uint32_t)(HAL_GetTick() - t_last_100) >= 10u)
    {
      t_last_100 += 10u;

      const float k = 5.0f;
      const float kx = 7.0f;

      const float POS_STEP_MAX_V = 50.0f;
      const float KP_Y = 10.0f;
      const float KP_Z = 10.0f;
      const float KP_X = 10.0f;
      const float QD_MAX = 5000.0f;
      const float VMAX_MOTOR = 3000.0f;

      if (ctrl_mode == CTRL_RUNNING && cmd_timeout_expired())
      {
        enter_comm_lost_stop();
      }

      // Leer posiciones reales
      int32_t p4 = Encoders_GetPos(0);
      int32_t p5 = Encoders_GetPos(1);
      int32_t p6 = Encoders_GetPos(2);

      float qY = 0.5f * ((float)p4 + (float)p5);
      float qZ = 0.5f * ((float)p4 - (float)p5);
      float qX = (float)p6;

      // Solo en RUNNING se aceptan comandos nuevos desde ROS2.
      // En SAFE_HOLD, spY_ref/spZ_ref/spX_ref quedan congelados.
      if (ctrl_mode == CTRL_RUNNING)
      {
        // Medición de X para compensación de acople
        float x_cdeg_meas = counts_to_cdeg_motor((float)p6);

        const float A_Y = 0.0f;
        //const float A_Z = 1/9; //0.14444f;
        const float A_Z = 1.0f / 12.0f; //0.14444f;

        // Compensar acople
        float y_eff_cdeg = (float)cmd_cdeg[0] - A_Y * x_cdeg_meas;
        float z_eff_cdeg = (float)cmd_cdeg[1] - A_Z * x_cdeg_meas;

        z_eff_cdeg_dbg = z_eff_cdeg;

        // Setpoints virtuales
        float qY_cmd = k * cdeg_to_counts_motor(y_eff_cdeg);
        float qZ_cmd = k * cdeg_to_counts_motor(z_eff_cdeg);
        float qX_cmd = kx * cdeg_to_counts_motor((float)cmd_cdeg[2]);

        float diffY = qY_cmd - spY_ref;
        float diffZ = qZ_cmd - spZ_ref;
        float diffX = qX_cmd - spX_ref;

        diffY = clampf(diffY, -POS_STEP_MAX_V, +POS_STEP_MAX_V);
        diffZ = clampf(diffZ, -POS_STEP_MAX_V, +POS_STEP_MAX_V);
        diffX = clampf(diffX, -POS_STEP_MAX_V, +POS_STEP_MAX_V);

        spY_ref += diffY;
        spZ_ref += diffZ;
        spX_ref += diffX;
      }

      // Este control se ejecuta tanto en RUNNING como en SAFE_HOLD.
      // En SAFE_HOLD, las referencias quedaron congeladas.
      if (ctrl_mode == CTRL_RUNNING || ctrl_mode == CTRL_SAFE_HOLD)
      {
        float qdY_ref = KP_Y * (spY_ref - qY);
        float qdZ_ref = KP_Z * (spZ_ref - qZ);
        float qdX_ref = KP_X * (spX_ref - qX);

        qdY_ref = clampf(qdY_ref, -QD_MAX, +QD_MAX);
        qdZ_ref = clampf(qdZ_ref, -QD_MAX, +QD_MAX);
        qdX_ref = clampf(qdX_ref, -QD_MAX, +QD_MAX);

        float v4_ref = (qdY_ref + qdZ_ref) / k;
        float v5_ref = (qdY_ref - qdZ_ref) / k;
        float v6_ref = qdX_ref / kx;

        // Saturación global v4/v5/v6
        float max_abs = fabsf(v4_ref);

        if (fabsf(v5_ref) > max_abs) max_abs = fabsf(v5_ref);
        if (fabsf(v6_ref) > max_abs) max_abs = fabsf(v6_ref);

        if (max_abs > VMAX_MOTOR)
        {
          float s = VMAX_MOTOR / max_abs;
          v4_ref *= s;
          v5_ref *= s;
          v6_ref *= s;
        }

        v_ref_motor[0] = v4_ref;
        v_ref_motor[1] = v5_ref;
        v_ref_motor[2] = v6_ref;
      }
      else
      {
        // CTRL_WAITING_CMD
        v_ref_motor[0] = 0.0f;
        v_ref_motor[1] = 0.0f;
        v_ref_motor[2] = 0.0f;
      }

      // Debug publish
      enc_buf[0] = p4;
      enc_buf[1] = p5;
      enc_buf[2] = p6;
      (void)rcl_publish(&pub_enc, &msg_enc, NULL);
      msg_z_eff.data = z_eff_cdeg_dbg;
      (void)rcl_publish(&pub_z_eff, &msg_z_eff, NULL);
    }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;

  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;

  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;

  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4199;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim5);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  emergency_outputs_off();

  __disable_irq();

  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}

#endif


