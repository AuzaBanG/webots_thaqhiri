import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Imu


class ImuMergerNode(Node):
    def __init__(self):
        super().__init__('imu_merger_node')

        # Ajusta estos nombres según los tópicos reales que expone tu robot en Webots
        accel_topic = '/drone/cube/acc'
        gyro_topic = '/drone/cube/gyro'
        imu_out_topic = '/imu/data_raw'

        # Suscriptores usando message_filters para la sincronización
        self.accel_sub = message_filters.Subscriber(self, Imu, accel_topic)
        self.gyro_sub = message_filters.Subscriber(self, Imu, gyro_topic)

        # ApproximateTimeSynchronizer empareja mensajes que llegan casi al mismo tiempo.
        # queue_size = 10, slop = 0.05 segundos (50ms de tolerancia)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.accel_sub, self.gyro_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.sync_callback)

        # Publicador del tópico combinado
        self.imu_pub = self.create_publisher(Imu, imu_out_topic, 10)
        self.get_logger().info('IMU Merger Node iniciado. Esperando datos sincronizados...')

    def sync_callback(self, accel_msg: Imu, gyro_msg: Imu):
        # Crear el mensaje Imu consolidado
        merged_imu = Imu()

        # 1. Asignar el Header (usamos el del acelerómetro como base)
        merged_imu.header = accel_msg.header

        # 2. Extraer Aceleración Lineal del tópico del acelerómetro
        # Webots publica la aceleración en el campo linear_acceleration
        merged_imu.linear_acceleration.x = accel_msg.linear_acceleration.x
        merged_imu.linear_acceleration.y = accel_msg.linear_acceleration.y
        merged_imu.linear_acceleration.z = accel_msg.linear_acceleration.z

        # 3. Extraer Velocidad Angular del tópico del giroscopio
        # Webots publica la velocidad angular en el campo angular_velocity
        merged_imu.angular_velocity.x = gyro_msg.angular_velocity.x
        merged_imu.angular_velocity.y = gyro_msg.angular_velocity.y
        merged_imu.angular_velocity.z = gyro_msg.angular_velocity.z

        # 4. Matrices de covarianza (Opcional, pero recomendado)
        # Si tienes ruidos conocidos de tus sensores simulados, puedes añadirlos aquí.
        # Dejaremos -1 en el primer elemento de la covarianza de orientación para indicar
        # que este mensaje no provee orientación (eso lo hará Madgwick después).
        merged_imu.orientation_covariance[0] = -1.0

        # Publicar el mensaje unificado
        self.imu_pub.publish(merged_imu)


def main(args=None):
    rclpy.init(args=args)
    node = ImuMergerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
