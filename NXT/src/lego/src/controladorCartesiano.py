#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import pinocchio as pin
import math
import numpy as np

from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class GeneradorTrayectoriaZ_g(Node):
    def __init__(self):
        super().__init__('trayectoria_posicion_Z')

        self.urdf_path = '/root/ros_ws/src/lego/urdf/Ensamblaje2.urdf'
        self.tip_link = 'Pinza'

        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(self.tip_link)

        self.joint_order = ['Joint1', 'Joint2', 'Joint3','Joint4', 'Joint5', 'Joint6']

        self.pos = None
        self.oMf_inicio = None

        self.t = 0.0
        self.dt = 0.02
        self.duracion = 4.0
        self.distancia_z = -0.20
        self.coef = np.zeros(6)
        self.ganancia_pos = 1.0
        self.ganancia_ori = 2.0
        self.damping = 1e-3
        self.qdot_max = 20

        self.joint_sub = self.create_subscription(JointState,'/joint_states',self.joint_state_callback,10)
        self.traj_pub = self.create_publisher(JointTrajectory,'/arm_controller/joint_trajectory',10)

        self.timer = self.create_timer(self.dt, self.loop_control)

    def joint_state_callback(self, msg):
        self.pos = dict(zip(msg.name, msg.position))

    def armar_q_full(self):
        q_arm = np.array([self.pos[joint_name] for joint_name in self.joint_order],dtype=float)
        q_full = np.zeros(self.model.nq)
        q_full[0:6] = q_arm
        return q_arm, q_full

    def publicar_q(self, q_cmd_arm):
        traj = JointTrajectory()
        traj.joint_names = self.joint_order

        point = JointTrajectoryPoint()
        point.positions = q_cmd_arm.tolist()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(self.dt * 1e9)

        traj.points.append(point)
        self.traj_pub.publish(traj)

    def Z_parametrizado(self, tiempo, z1, z2): #podria ser una recta, la derivda de eso es la velocidad

        # Tiempo inicial y final
        ti = 0.0
        tf = tiempo   # segundos

        # Condiciones iniciales
        pi = z1       # posición inicial
        vi = 0.0       # velocidad inicial
        acci = 0.0     # aceleración inicial

        # Condiciones finales
        pf = z2       # posición final
        vf = 0.0       # velocidad final
        accf = 0.0     # aceleración final

        # Sistema A @ coef = b
        A = np.array([
            [1, ti, ti**2, ti**3, ti**4, ti**5],                    # p(ti)
            [0, 1,  2*ti,  3*ti**2, 4*ti**3, 5*ti**4],              # p'(ti)
            [0, 0,  2,     6*ti,    12*ti**2, 20*ti**3],            # p''(ti)

            [1, tf, tf**2, tf**3, tf**4, tf**5],                    # p(tf)
            [0, 1,  2*tf,  3*tf**2, 4*tf**3, 5*tf**4],              # p'(tf)
            [0, 0,  2,     6*tf,    12*tf**2, 20*tf**3],            # p''(tf)
        ], dtype=float)

        b = np.array([
            pi,
            vi,
            acci,
            pf,
            vf,
            accf
        ], dtype=float)

        return np.linalg.solve(A, b) # coeficientes ordenados de menor a manyor
    
    def Z_parametrizado_vel_max(self, x0, xf, vmax):

        D = xf - x0
        if abs(D) < 1e-12:
            coef = np.array([x0, 0.0, 0.0, 0.0, 0.0, 0.0])
            T = 0.0
            return coef, T

        if vmax <= 0:
            raise ValueError("vmax debe ser mayor que cero")

        # Para el quintico normalizado, la velocidad máxima es:
        # vmax = 1.875 * |D| / T
        T = 1.875 * abs(D) / vmax

        a0 = x0
        a1 = 0.0
        a2 = 0.0
        a3 = 10.0 * D / T**3
        a4 = -15.0 * D / T**4
        a5 = 6.0 * D / T**5

        coef = np.array([a0, a1, a2, a3, a4, a5])

        return coef

    def parametrizacion_posZ(self, t):
        return sum(self.coef[i] * t**i for i in range(len(self.coef)))
    
    def parametrizacion_velZ(self, t):
        return sum(i * self.coef[i] * t**(i - 1) for i in range(1, len(self.coef)))

    def loop_control(self):
        if self.pos is None:
            return

        if self.t > self.duracion:
            self.get_logger().info('Trayectoria finalizada')
            self.timer.cancel()
            return

        q_arm, q_full = self.armar_q_full()

        pin.forwardKinematics(self.model, self.data, q_full)
        pin.computeJointJacobians(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)

        oMf_actual = self.data.oMf[self.frame_id]

        if self.oMf_inicio is None:
            self.oMf_inicio = pin.SE3(oMf_actual)
            self.get_logger().info('Pose inicial guardada.')
            #self.coef = np.array(self.Z_parametrizado(self.duracion, 0, self.distancia_z))
            self.coef = np.array(self.Z_parametrizado_vel_max(0, self.distancia_z, 0.1))
            
            
        Zpos = self.parametrizacion_posZ(self.t)
        Zvel = self.parametrizacion_velZ(self.t)
        
        oMf_deseada = pin.SE3(self.oMf_inicio) 
        
        oMf_deseada.translation[2] = (self.oMf_inicio.translation[2] + Zpos)
    
        err_pos = oMf_deseada.translation - oMf_actual.translation
        
        R_error = oMf_deseada.rotation @ oMf_actual.rotation.T
        err_ori = pin.log3(R_error)                          #Convierte ese "error" en velocidades de cada eje

        twist_error = np.concatenate([err_pos, err_ori])

        J = pin.getFrameJacobian(self.model,self.data,self.frame_id,pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J_arm = J[:, 0:6]

        K = np.diag([
            self.ganancia_pos,
            self.ganancia_pos,
            self.ganancia_pos,
            self.ganancia_ori,
            self.ganancia_ori,
            self.ganancia_ori
        ])

        twist_ref = np.array([
            0.0,
            0.0,
            Zvel,
            0.0,
            0.0,
            0.0
        ])
        twist_cmd = twist_ref + K @ twist_error


        A = J_arm @ J_arm.T + self.damping * np.eye(6)

        dq_arm = J_arm.T @ np.linalg.solve(A,K @ twist_cmd)
        
        dq_arm = np.clip(dq_arm, -self.qdot_max, self.qdot_max)

        dq_full = np.zeros(self.model.nv)
        dq_full[0:6] = dq_arm

        q_next_full = pin.integrate(self.model,q_full,dq_full * self.dt)

        q_cmd_arm = q_next_full[0:6]
        
        self.publicar_q(q_cmd_arm)

        #self.get_logger().info(
        #   f"x_err={err_pos[0]:.4f}, "
        #    f"y_err={err_pos[1]:.4f}, "
        #    f"z_err={err_pos[2]:.4f}, "
        #    f"ori_err_norm={np.linalg.norm(err_ori):.4f}"            
        #)
        
        self.t += self.dt

class GeneradorTrayectoriaX_g(Node):
    def __init__(self):
        super().__init__('trayectoria_posicion_Z')

        self.urdf_path = '/root/ros_ws/src/lego/urdf/Ensamblaje2.urdf'
        self.tip_link = 'Pinza'

        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(self.tip_link)

        self.joint_order = ['Joint1', 'Joint2', 'Joint3','Joint4', 'Joint5', 'Joint6']

        self.pos = None
        self.oMf_inicio = None

        self.amplitud = math.radians(10.0)   # +/- 10 grados
        self.frecuencia = 0.3                # Hz
        self.t = 0.0
        self.dt = 0.02
        self.duracion = 4.0
        self.distancia_x = 0.30
        self.coef = np.zeros(6)
        self.ganancia_pos = 1.0
        self.ganancia_ori = 2.0
        self.damping = 1e-3
        self.qdot_max = 20

        self.joint_sub = self.create_subscription(JointState,'/joint_states',self.joint_state_callback,10)
        self.traj_pub = self.create_publisher(JointTrajectory,'/arm_controller/joint_trajectory',10)

        self.timer = self.create_timer(self.dt, self.loop_control)

    def joint_state_callback(self, msg):
        self.pos = dict(zip(msg.name, msg.position))

    def armar_q_full(self):
        q_arm = np.array([self.pos[joint_name] for joint_name in self.joint_order],dtype=float)
        q_full = np.zeros(self.model.nq)
        q_full[0:6] = q_arm
        return q_arm, q_full

    def publicar_q(self, q_cmd_arm):
        traj = JointTrajectory()
        traj.joint_names = self.joint_order

        point = JointTrajectoryPoint()
        point.positions = q_cmd_arm.tolist()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(self.dt * 1e9)

        traj.points.append(point)
        self.traj_pub.publish(traj)

    def X_parametrizado(self, tiempo, x1, x2): #podria ser una recta, la derivda de eso es la velocidad

        # Tiempo inicial y final
        ti = 0.0
        tf = tiempo   # segundos

        # Condiciones iniciales
        pi = x1       # posición inicial
        vi = 0.0       # velocidad inicial
        acci = 0.0     # aceleración inicial

        # Condiciones finales
        pf = x2       # posición final
        vf = 0.0       # velocidad final
        accf = 0.0     # aceleración final

        # Sistema A @ coef = b
        A = np.array([
            [1, ti, ti**2, ti**3, ti**4, ti**5],                    # p(ti)
            [0, 1,  2*ti,  3*ti**2, 4*ti**3, 5*ti**4],              # p'(ti)
            [0, 0,  2,     6*ti,    12*ti**2, 20*ti**3],            # p''(ti)

            [1, tf, tf**2, tf**3, tf**4, tf**5],                    # p(tf)
            [0, 1,  2*tf,  3*tf**2, 4*tf**3, 5*tf**4],              # p'(tf)
            [0, 0,  2,     6*tf,    12*tf**2, 20*tf**3],            # p''(tf)
        ], dtype=float)

        b = np.array([
            pi,
            vi,
            acci,
            pf,
            vf,
            accf
        ], dtype=float)

        return np.linalg.solve(A, b) # coeficientes ordenados de menor a manyor

    def parametrizacion_posX(self, t):
        return sum(self.coef[i] * t**i for i in range(len(self.coef)))
    
    def parametrizacion_velX(self, t):
        return sum(i * self.coef[i] * t**(i - 1) for i in range(1, len(self.coef)))

    def loop_control(self):
        if self.pos is None:
            return

        if self.t > self.duracion:
            self.get_logger().info('Trayectoria finalizada')
            self.timer.cancel()
            return

        q_arm, q_full = self.armar_q_full()

        pin.forwardKinematics(self.model, self.data, q_full)
        pin.computeJointJacobians(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)

        oMf_actual = self.data.oMf[self.frame_id]

        if self.oMf_inicio is None:
            self.oMf_inicio = pin.SE3(oMf_actual)
            self.get_logger().info('Pose inicial guardada.')
            self.coef = np.array(self.X_parametrizado(self.duracion, 0, self.distancia_x))
            #self.coef = np.array(self.X_parametrizado_vel_max(0, self.distancia_x, 0.1))
            
            
        Xpos = self.parametrizacion_posX(self.t)
        Xvel = self.parametrizacion_velX(self.t)
        

        oMf_deseada = pin.SE3(self.oMf_inicio) 
        
        oMf_deseada.translation[0] = (self.oMf_inicio.translation[0] + Xpos)
    
        err_pos = oMf_deseada.translation - oMf_actual.translation
        
        
        R_error = oMf_deseada.rotation @ oMf_actual.rotation.T
        err_ori = pin.log3(R_error)                          #Convierte ese "error" en velocidades de cada eje

        twist_error = np.concatenate([err_pos, err_ori])

        J = pin.getFrameJacobian(self.model,self.data,self.frame_id,pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J_arm = J[:, 0:6]

        K = np.diag([
            self.ganancia_pos,
            self.ganancia_pos,
            self.ganancia_pos,
            self.ganancia_ori,
            self.ganancia_ori,
            self.ganancia_ori
        ])

        twist_ref = np.array([
            Xvel,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ])
        twist_cmd = twist_ref + K @ twist_error
        

        A = J_arm @ J_arm.T + self.damping * np.eye(6)

        dq_arm = J_arm.T @ np.linalg.solve(A,K @ twist_cmd)
        
        dq_arm = np.clip(dq_arm, -self.qdot_max, self.qdot_max)

        dq_full = np.zeros(self.model.nv)
        dq_full[0:6] = dq_arm

        q_next_full = pin.integrate(self.model,q_full,dq_full * self.dt)

        q_cmd_arm = q_next_full[0:6]
        
        self.publicar_q(q_cmd_arm)


        
        self.t += self.dt

class GeneradorTrayectoriaY_g(Node):
    def __init__(self):
        super().__init__('trayectoria_posicion_Y')

        # Se asume la misma ruta al URDF y el mismo frame_id ("Pinza") [cite: 2011, 2014]
        self.urdf_path = '/root/ros_ws/src/lego/urdf/Ensamblaje2.urdf'
        self.tip_link = 'Pinza'

        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(self.tip_link)

        self.joint_order = ['Joint1', 'Joint2', 'Joint3','Joint4', 'Joint5', 'Joint6']

        self.pos = None
        self.oMf_inicio = None

        self.t = 0.0
        self.dt = 0.02
        self.duracion = 4.0
        self.distancia_y = 0.20  # Desplazamiento deseado en Y (modificar según necesidad)
        self.coef = np.zeros(6)
        self.ganancia_pos = 1.0
        self.ganancia_ori = 2.0
        self.damping = 1e-3
        self.qdot_max = 20

        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.traj_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self.timer = self.create_timer(self.dt, self.loop_control)

    def joint_state_callback(self, msg):
        self.pos = dict(zip(msg.name, msg.position))

    def armar_q_full(self):
        q_arm = np.array([self.pos[joint_name] for joint_name in self.joint_order], dtype=float)
        q_full = np.zeros(self.model.nq)
        q_full[0:6] = q_arm
        return q_arm, q_full

    def publicar_q(self, q_cmd_arm):
        traj = JointTrajectory()
        traj.joint_names = self.joint_order

        point = JointTrajectoryPoint()
        point.positions = q_cmd_arm.tolist()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(self.dt * 1e9)

        traj.points.append(point)
        self.traj_pub.publish(traj)

    def Y_parametrizado(self, tiempo, y1, y2): 
        # Tiempo inicial y final
        ti = 0.0
        tf = tiempo   # segundos

        # Condiciones iniciales
        pi = y1       # posición inicial
        vi = 0.0      # velocidad inicial
        acci = 0.0    # aceleración inicial

        # Condiciones finales
        pf = y2       # posición final
        vf = 0.0      # velocidad final
        accf = 0.0    # aceleración final

        # Sistema A @ coef = b
        A = np.array([
            [1, ti, ti**2, ti**3, ti**4, ti**5],                    
            [0, 1,  2*ti,  3*ti**2, 4*ti**3, 5*ti**4],              
            [0, 0,  2,     6*ti,    12*ti**2, 20*ti**3],            

            [1, tf, tf**2, tf**3, tf**4, tf**5],                    
            [0, 1,  2*tf,  3*tf**2, 4*tf**3, 5*tf**4],              
            [0, 0,  2,     6*tf,    12*tf**2, 20*tf**3],            
        ], dtype=float)

        b = np.array([
            pi, vi, acci,
            pf, vf, accf
        ], dtype=float)

        return np.linalg.solve(A, b) 

    def parametrizacion_posY(self, t):
        return sum(self.coef[i] * t**i for i in range(len(self.coef)))
    
    def parametrizacion_velY(self, t):
        return sum(i * self.coef[i] * t**(i - 1) for i in range(1, len(self.coef)))

    def loop_control(self):
        if self.pos is None:
            return

        if self.t > self.duracion:
            self.get_logger().info('Trayectoria en Y finalizada')
            self.timer.cancel()
            return

        q_arm, q_full = self.armar_q_full()

        pin.forwardKinematics(self.model, self.data, q_full)
        pin.computeJointJacobians(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)

        oMf_actual = self.data.oMf[self.frame_id]

        if self.oMf_inicio is None:
            self.oMf_inicio = pin.SE3(oMf_actual)
            self.get_logger().info('Pose inicial guardada para movimiento en Y.')
            self.coef = np.array(self.Y_parametrizado(self.duracion, 0, self.distancia_y))
            
        Ypos = self.parametrizacion_posY(self.t)
        Yvel = self.parametrizacion_velY(self.t)

        oMf_deseada = pin.SE3(self.oMf_inicio) 
        
        # Desplazamiento aplicado al índice 1 (Eje Y)
        oMf_deseada.translation[1] = (self.oMf_inicio.translation[1] + Ypos)
    
        err_pos = oMf_deseada.translation - oMf_actual.translation
        
        R_error = oMf_deseada.rotation @ oMf_actual.rotation.T
        err_ori = pin.log3(R_error) 

        twist_error = np.concatenate([err_pos, err_ori])

        J = pin.getFrameJacobian(self.model, self.data, self.frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J_arm = J[:, 0:6]

        K = np.diag([
            self.ganancia_pos,
            self.ganancia_pos,
            self.ganancia_pos,
            self.ganancia_ori,
            self.ganancia_ori,
            self.ganancia_ori
        ])

        # Velocidad inyectada en el índice 1 (Eje Y)
        twist_ref = np.array([
            0.0,
            Yvel,
            0.0,
            0.0,
            0.0,
            0.0
        ])
        
        twist_cmd = twist_ref + K @ twist_error

        # Control cinemático usando pseudoinversa amortiguada
        A = J_arm @ J_arm.T + self.damping * np.eye(6)
        dq_arm = J_arm.T @ np.linalg.solve(A, K @ twist_cmd)
        
        dq_arm = np.clip(dq_arm, -self.qdot_max, self.qdot_max)

        dq_full = np.zeros(self.model.nv)
        dq_full[0:6] = dq_arm

        q_next_full = pin.integrate(self.model, q_full, dq_full * self.dt)
        q_cmd_arm = q_next_full[0:6]
        
        self.publicar_q(q_cmd_arm)
        
        self.t += self.dt

class GeneradorRotacionY_g(Node):

    def __init__(self):
        super().__init__('Rotacion_y')

        self.urdf_path = '/root/ros_ws/src/lego/urdf/Ensamblaje2.urdf'
        self.tip_link = 'Pinza'

        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(self.tip_link)
        
        self.joint_order = ['Joint1', 'Joint2', 'Joint3','Joint4', 'Joint5', 'Joint6']

        self.pos = None
        self.oMf_inicio = None

        self.amplitud = math.radians(10.0)   # +/- 10 grados
        self.frecuencia =  0.5               # Hz
        self.t = 0.0
        self.dt = 0.02
        self.duracion = 4.0
        self.distancia_x = -0.40
        self.coef = np.zeros(6)
        self.ganancia_pos = 1.0
        self.ganancia_ori = 2.0
        self.damping = 1e-3
        self.qdot_max = 20
        
        self.joint_sub = self.create_subscription(JointState,'/joint_states',self.joint_state_callback,10)
        self.traj_pub = self.create_publisher(JointTrajectory,'/arm_controller/joint_trajectory',10)
        
        self.timer = self.create_timer(self.dt, self.control_loop)
    
    def joint_state_callback(self, msg):
        self.pos = dict(zip(msg.name, msg.position))

    def armar_q_full(self):
        q_arm = np.array([self.pos[joint_name] for joint_name in self.joint_order],dtype=float)
        q_full = np.zeros(self.model.nq)
        q_full[0:6] = q_arm
        return q_arm, q_full    

    def joint_state_callback(self, msg):
        self.pos = dict(zip(msg.name, msg.position))

    def publicar_q(self, q_cmd_arm):
        traj = JointTrajectory()
        traj.joint_names = self.joint_order

        point = JointTrajectoryPoint()
        point.positions = q_cmd_arm.tolist()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(self.dt * 1e9)

        traj.points.append(point)
        self.traj_pub.publish(traj)

    def control_loop(self):
        
        if self.pos is None:
            return
        
        q_arm, q_full = self.armar_q_full()
                
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.computeJointJacobians(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)

        oMf_actual = self.data.oMf[self.frame_id]

        if self.oMf_inicio is None:
            self.oMf_inicio = oMf_actual.copy()
            self.get_logger().info('Pose inicial guardada.')

        
        theta = self.amplitud * math.sin(2.0 * math.pi * self.frecuencia * self.t)
        theta_dot = (self.amplitud* 2.0* math.pi* self.frecuencia* math.cos(2.0 * math.pi * self.frecuencia * self.t))

        R_inicio = self.oMf_inicio.rotation
        R_oscilacion_local = pin.utils.rotate('y', theta)
        #R_deseada = R_inicio @ R_oscilacion_local #local
        R_deseada = R_oscilacion_local @ R_inicio #Global
        error_orientacion = pin.log3(oMf_actual.rotation.T @ R_deseada)
        w_feedforward = oMf_actual.rotation.T @ (R_inicio @ np.array([0.0, theta_dot, 0.0]))
        Kp_w = 20.0  # Ajusta según tu sistema
        w_cmd = w_feedforward + Kp_w * error_orientacion

        p_actual = oMf_actual.translation
        p_inicio = self.oMf_inicio.translation
        error_posicion = oMf_actual.rotation.T @ (p_inicio - p_actual)

        Kp_p = 20.0 
        v_cmd = Kp_p * error_posicion
        
        twist_cmd = np.concatenate([v_cmd, w_cmd])
        J = pin.getFrameJacobian(self.model, self.data, self.frame_id, pin.ReferenceFrame.LOCAL)
        num_dof_brazo = len(q_arm) 
        J_arm = J[:, :num_dof_brazo]
        damping = 1e-4
        J_inv = np.linalg.pinv(J_arm.T @ J_arm + damping * np.eye(num_dof_brazo)) @ J_arm.T
        
        # Velocidades articulares calculadas (rad/s)
        qdot_cmd = J_inv @ twist_cmd
        dq = qdot_cmd * self.dt


        qdot_full = np.zeros(self.model.nv)
        num_dof_brazo = len(q_arm)
        qdot_full[:num_dof_brazo] = qdot_cmd


        q_full_siguiente = pin.integrate(self.model, q_full, qdot_full * self.dt)
        q_cmd_arm = q_full_siguiente[:num_dof_brazo]
        self.publicar_q(q_cmd_arm)
        self.t += self.dt

class GeneradorTrayectoriaZ_r(Node):
    def __init__(self):
        super().__init__('trayectoria_posicion_Z_r')

        self.urdf_path = '/root/ros_ws/src/lego/urdf/Ensamblaje2.urdf'
        self.tip_link = 'Pinza'
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(self.tip_link)
        self.q_ini = pin.neutral(self.model)

        self.t = 0.0
        self.dt = 0.02
        self.duracion = 4.0
        self.distancia_z1 = 0.0
        self.distancia_z2 = -0.20
        self.coef = np.zeros(6)        
        self.damping = 1e-3
        self.coef0 = 0 
        self.coef1 = 0 
        self.velocidad_TCP = np.zeros(6)
        self.traj_client = None
        Q_ = self.generador_puntos()
        self.crear_action_client()
        self.enviar_trayectoria_al_bridge(Q_)

    def crear_action_client(self):
        self.traj_client = ActionClient(self,FollowJointTrajectory,'/BRAZO_controller/follow_joint_trajectory')  

    def enviar_trayectoria_al_bridge(self, Q_):
        if len(Q_) == 0:
            self.get_logger().error('La trayectoria Q_ está vacía.')
            return

        self.get_logger().info('Esperando action server /BRAZO_controller/follow_joint_trajectory...')

        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('No encontré el action server del bridge.')
            return

        goal_msg = FollowJointTrajectory.Goal()

        goal_msg.trajectory.joint_names = [
            'Joint1',
            'Joint2',
            'Joint3',
            'Joint4',
            'Joint5',
            'Joint6'
        ]

        for i, q_full in enumerate(Q_):
            punto = JointTrajectoryPoint()

            # Si tu modelo tiene 8 joints, usamos solo los primeros 6 del brazo.
            q_arm = q_full[:6]

            punto.positions = [
                float(q_arm[0]),
                float(q_arm[1]),
                float(q_arm[2]),
                float(q_arm[3]),
                float(q_arm[4]),
                float(q_arm[5]),
            ]

            t = i * self.dt
            sec = int(t)
            nanosec = int((t - sec) * 1e9)

            punto.time_from_start = Duration(
                sec=sec,
                nanosec=nanosec
            )

            goal_msg.trajectory.points.append(punto)

        self.get_logger().info(
            f'Enviando trayectoria con {len(goal_msg.trajectory.points)} puntos al bridge.'
        )

        future = self.traj_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)  
    
    
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('El bridge rechazó la trayectoria.')
            return

        self.get_logger().info('El bridge aceptó la trayectoria.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)


    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Trayectoria terminada por el bridge.')
    
    def Z_parametrizado(self, tiempo, z1, z2): #podria ser una recta, la derivda de eso es la velocidad

        # Tiempo inicial y final
        ti = 0.0
        tf = tiempo   # segundos

        # Condiciones iniciales
        pi = z1       # posición inicial
        vi = 0.0       # velocidad inicial
        acci = 0.0     # aceleración inicial

        # Condiciones finales
        pf = z2       # posición final
        vf = 0.0       # velocidad final
        accf = 0.0     # aceleración final

        # Sistema A @ coef = b
        A = np.array([
            [1, ti, ti**2, ti**3, ti**4, ti**5],                    # p(ti)
            [0, 1,  2*ti,  3*ti**2, 4*ti**3, 5*ti**4],              # p'(ti)
            [0, 0,  2,     6*ti,    12*ti**2, 20*ti**3],            # p''(ti)

            [1, tf, tf**2, tf**3, tf**4, tf**5],                    # p(tf)
            [0, 1,  2*tf,  3*tf**2, 4*tf**3, 5*tf**4],              # p'(tf)
            [0, 0,  2,     6*tf,    12*tf**2, 20*tf**3],            # p''(tf)
        ], dtype=float)

        b = np.array([
            pi,
            vi,
            acci,
            pf,
            vf,
            accf
        ], dtype=float)

        return np.linalg.solve(A, b) # coeficientes ordenados de menor a manyor    

    def Z_parametrizado_vel_max(self, z0, zf, vmax):

        D = zf - z0
        if abs(D) < 1e-12:
            coef = np.array([z0, 0.0, 0.0, 0.0, 0.0, 0.0])
            T = 0.0
            return coef, T

        if vmax <= 0:
            raise ValueError("vmax debe ser mayor que cero")

        # Para el quintico normalizado, la velocidad máxima es:
        # vmax = 1.875 * |D| / T
        T = 1.875 * abs(D) / vmax

        a0 = z0
        a1 = 0.0
        a2 = 0.0
        a3 = 10.0 * D / T**3
        a4 = -15.0 * D / T**4
        a5 = 6.0 * D / T**5
        
        coef0 = np.array([a0, a1, a2, a3, a4, a5])
        coef1 = np.array([a1, 2*a2, 3*a3, 4*a4, 5*a5])
        T = np.roots(coef1[::-1])
        reales = np.sort(T[T.imag == 0].real)
        reales = reales[~np.isclose(reales, 0, atol=1e-9)]

        return coef0, coef1, reales
    
    
    def parametrizacion_posZ(self, t):
        return sum(self.coef0[i] * t**i for i in range(len(self.coef0)))
    
    def parametrizacion_velZ(self, t):
        return sum(self.coef1[i] * t**i for i in range(len(self.coef1)))
        #return sum(i * self.coef[i] * t**(i - 1) for i in range(1, len(self.coef)))

    def J_inv(self, J):
        J = np.asarray(J, dtype=float)

        m, n = J.shape
        lambda2 = self.damping ** 2
        J_pinv = J.T @ np.linalg.inv(J @ J.T + lambda2 * np.eye(m))

        return J_pinv
        
    def guardar_Q_en_txt(self, Q_, nombre_archivo="Q_datos.txt"):
        with open(nombre_archivo, "w") as archivo:
            for q in Q_:
                archivo.write(str(q) + "\n")    

    def generador_puntos(self):
        Q_ = []
        Q = self.q_ini
        self.coef0, self.coef1, T = self.Z_parametrizado_vel_max(self.distancia_z1, self.distancia_z2, 0.1) #pol, poldev, tiempo
        N=int(round(T[0]/self.dt))
        
        for t in range(N):
            self.velocidad_TCP = np.zeros(6)
            self.velocidad_TCP[2] = self.parametrizacion_velZ(self.t)

            pin.forwardKinematics(self.model, self.data, Q)
            pin.computeJointJacobians(self.model, self.data, Q)
            pin.updateFramePlacements(self.model, self.data)

            J = pin.computeFrameJacobian(self.model, self.data, Q, self.frame_id,  pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            Q_punto = self.J_inv(J)@self.velocidad_TCP
            q_next_full = pin.integrate(self.model,Q , Q_punto * self.dt)
            Q = q_next_full
            Q_.append(Q.copy())
            
            self.t += self.dt
        print(Q_) 
       
        return Q_


def main(args=None):
    rclpy.init(args=args)
    #node = GeneradorTrayectoriaZ_g()
    #node = GeneradorTrayectoriaX_g()
    node = GeneradorTrayectoriaY_g()

    #node = GeneradorRotacionY_g()
    #node = GeneradorTrayectoriaZ_r()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
