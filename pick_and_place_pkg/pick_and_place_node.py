import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray, Bool
from builtin_interfaces.msg import Duration

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        
        # Cliente de acción para el brazo UR5
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Publicador para el gripper
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, 
            '/gripper_controller/commands', 
            10
        )
        
        # Suscriptor para simular la entrada digital
        self.digital_input = False
        self.sub_digital = self.create_subscription(
            Bool, 
            '/digital_input', 
            self.digital_input_cb, 
            10
        )
        
        # Articulaciones del UR5
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # Posiciones de la rutina
        self.p_home = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.p_approach = [1.0, -1.0, 1.2, -1.57, -1.57, 0.0]
        self.p_pick = [1.0, -0.8, 1.4, -1.57, -1.57, 0.0]
        self.p_place = [-1.0, -1.0, 1.2, -1.57, -1.57, 0.0]

        # Secuencia ordenada de tareas: ('TIPO', datos, duración)
        self.sequence = [
            ('GRIPPER', [0.01, 0.01], 1.0),
            ('ARM', self.p_home, 3.0),
            ('ARM', self.p_approach, 3.0),
            ('ARM', self.p_pick, 3.0),
            ('GRIPPER', [0.0, 0.0], 1.0),
            ('ARM', self.p_approach, 3.0),
            ('ARM', self.p_place, 3.0),
            ('GRIPPER', [0.01, 0.01], 1.0),
            ('ARM', self.p_home, 3.0)
        ]
        
        self.step_index = 0
        self.state = 'WAITING_TRIGGER'
        self.active_future = None
        self.get_result_future = None
        self.action_in_progress = False
        self.gripper_timer = None

        self.get_logger().info('Nodo Pick and Place asíncrono listo. Esperando señal en /digital_input (True)...')
        
        # Bucle de control periódico no bloqueante (cada 0.1 segundos)
        self.timer = self.create_timer(0.1, self.control_loop)

    def digital_input_cb(self, msg):
        self.digital_input = msg.data
        if self.digital_input and self.state == 'WAITING_TRIGGER':
            self.get_logger().info('¡Señal digital recibida! Iniciando rutina...')
            self.step_index = 0
            self.state = 'EXECUTING_ROUTINE'

    def control_loop(self):
        if self.state == 'WAITING_TRIGGER':
            return
        
        elif self.state == 'EXECUTING_ROUTINE':
            if self.action_in_progress:
                return  # Esperando a que el paso actual termine

            if self.step_index >= len(self.sequence):
                self.get_logger().info('Rutina completada. Esperando nueva señal digital...')
                self.state = 'WAITING_TRIGGER'
                self.digital_input = False
                return

            task_type, data, duration = self.sequence[self.step_index]

            if task_type == 'GRIPPER':
                msg = Float64MultiArray()
                msg.data = data
                self.gripper_pub.publish(msg)
                self.get_logger().info(f'Gripper accionado con: {data}')
                self.action_in_progress = True
                self.gripper_timer = self.create_timer(duration, self.finish_gripper_action)

            elif task_type == 'ARM':
                self.send_arm_goal(data, duration)

        elif self.state == 'WAITING_ARM_RESULT':
            if self.active_future is not None and self.active_future.done():
                goal_handle = self.active_future.result()
                self.active_future = None
                if goal_handle.accepted:
                    self.get_logger().info('Objetivo aceptado. Esperando llegada al destino...')
                    self.get_result_future = goal_handle.get_result_async()
                    self.state = 'WAITING_ARM_FINAL_RESULT'
                else:
                    self.get_logger().error('El objetivo del brazo fue rechazado.')
                    self.action_in_progress = False
                    self.state = 'EXECUTING_ROUTINE'

        elif self.state == 'WAITING_ARM_FINAL_RESULT':
            if self.get_result_future is not None and self.get_result_future.done():
                self.get_logger().info('¡El brazo llegó a la posición!')
                self.get_result_future = None
                self.action_in_progress = False
                self.step_index += 1
                self.state = 'EXECUTING_ROUTINE'

    def finish_gripper_action(self):
        if self.gripper_timer is not None:
            self.gripper_timer.cancel()
            self.gripper_timer = None
        self.action_in_progress = False
        self.step_index += 1

    def send_arm_goal(self, positions, duration_sec):
        if not self._action_client.wait_for_server(timeout_sec=0.1):
            return  # Intentará de nuevo en el siguiente ciclo del timer

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        goal_msg.trajectory.points = [point]

        self.get_logger().info(f'Moviendo brazo a: {positions}')
        self.action_in_progress = True
        self.active_future = self._action_client.send_goal_async(goal_msg)
        self.state = 'WAITING_ARM_RESULT'

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
