import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class ClienteComandador(Node):
    def __init__(self):
        super().__init__('cliente_comandador')
        self.cli = self.create_client(SetBool, '/comandador')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /comandador...')
        self.get_logger().info("Cliente listo. Ingresá 0 o 1, o 'q' para salir.")

        self.loop_teclado()

    def loop_teclado(self):
        while rclpy.ok():
            comando = input("Comando (0/1): ")
            if comando.lower() == 'q':
                break
            if comando in ['0', '1']:
                valor = True if comando == '1' else False
                self.enviar_comando(valor)
            else:
                self.get_logger().warn("Entrada inválida. Ingrese 0 o 1.")

    def enviar_comando(self, valor):
        request = SetBool.Request()
        request.data = valor
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Respuesta: {future.result().message}")
        else:
            self.get_logger().error("Error al llamar al servicio.")

def main(args=None):
    rclpy.init(args=args)
    node = ClienteComandador()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()