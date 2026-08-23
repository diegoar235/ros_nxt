#!/usr/bin/env python3

import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Int32MultiArray
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
import json, os, math
from std_srvs.srv import SetBool  # Servicio simple: True/False


class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(-128, 127)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits

        # Variables internas
        self._prev_error = 0.0
        self._integral = 0.0

    def reset(self):
        """Reinicia las variables internas del PID."""
        self._prev_error = 0.0
        self._integral = 0.0

    def update(self, setpoint, measurement, dt):

        error = setpoint - measurement

        # Términos PID
        p = self.kp * error
        self._integral += error * dt
        i = self.ki * self._integral
        d = self.kd * (error - self._prev_error) / dt if dt > 0 else 0.0

        # PID total
        output = p + i + d

        if output > 95: #127:
            output = 95 #127
        if output < -95: #128:
            output = -95 #128

        # Guardar error previo
        self._prev_error = error

        return round(output)


class Robot_Control(Node):  #Control lazo del robot, contiene los PID y ganancias, se subscribe a moveit y al feedback del robot, publica en el nodo de las STM
    
    def __init__(self):
        super().__init__('Lazo_Control')                                                                                 #Nombre del nodo (Lazo_Control)
        self.publisher_ = self.create_publisher(Int8MultiArray, '/stm32/comando', 10)                                    #Pub los valores de cada motor                          
        self.subscription_ = self.create_subscription(Int32MultiArray,'/feedback_robot/datos',self.feedback_callback,10) #Subs a lo que devuelve el robot
        self.moveit_sub_ = self.create_subscription(JointState ,'/joint_states',self.moveit_callback,10)                 #Subs a moveit
        
        self.timer_ = self.create_timer(0.1, self.publicar_comando)                                                      #cada 100 msegundo
        self.valor_medido_juntas = [0.0] * 6
        self.valor_juntas_moveit = {}
        self.tengo_joint_states = False

        self.reset = 1
        config_path = os.path.join(get_package_share_directory('lego'), 'config','gearRatio.json')
        with open(config_path, 'r') as f:
            gears = json.load(f)
        self.pid1 = PIDController(kp=1.4, ki=0, kd=0.12)
        self.pid2 = PIDController(kp=1.4, ki=0, kd=0.12)
        self.pid3 = PIDController(kp=1.4, ki=0, kd=0.12)
            
    def feedback_callback(self, msg):
        self.valor_medido_juntas = msg.data
        return None
    
    def moveit_callback(self, msg):
        for name, pos_rad in zip(msg.name, msg.position):
            self.valor_juntas_moveit[name] = math.degrees(pos_rad)
        self.tengo_joint_states = True    
        return None

    '''
    def callback_comando(self, request, response):
        if request.data == 1:
            self.get_logger().info("Comando recibido: 1 (Activar)")
            response.success = True
            response.message = "Encoder activado"
            self.reset = 1
        if request.data == 0:
            self.get_logger().info("Comando recibido: 0 (Desactivar)")
            response.success = True
            response.message = "Encoder reset"
            self.reset = 0
        return response
    '''

    def publicar_comando(self): #Hago todo el control en deg
        if not self.tengo_joint_states:
            self.get_logger().warn("Aún no recibí /joint_states; no publico comando.")
            return
        
        j1 = self.pid1.update(self.valor_juntas_moveit['Joint1']*5, self.valor_medido_juntas[0], 0.1) # control junta 1
        j2 = self.pid2.update(self.valor_juntas_moveit['Joint2']*14.01, self.valor_medido_juntas[1], 0.1) # control junta 2
        j3 = self.pid3.update(self.valor_juntas_moveit['Joint3']*10.02, self.valor_medido_juntas[2], 0.1) # control junta 3
        self.get_logger().info(f"j2: {j2}")
        comando = [1,j1,j2,j3,0,0,0,1,1,1,1]
        msg = Int8MultiArray()
        msg.data = comando
        self.publisher_.publish(msg)

        
        

def main(args=None):
    rclpy.init(args=args)
    node = Robot_Control()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
