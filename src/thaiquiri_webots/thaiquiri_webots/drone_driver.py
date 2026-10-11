"""ROS 2 driver for the Thaiquiri quadrotor simulated in Webots."""

import rclpy
from geometry_msgs.msg import Twist
from webots_ros2_msgs.msg import FloatStamped
from sensor_msgs.msg import Imu, JointState
from time import sleep


class DroneDriver:
    """Translate ROS 2 flight commands into Webots motor velocities."""

    def init(self, webots_node, properties):
        """Initialize Webots devices, controller gains, and ROS interfaces.

        Args:
            webots_node: Webots ROS 2 node that owns the simulated robot.
            properties: Controller properties supplied by the Webots launcher.
                The current driver does not require any custom properties.
        """
        self.__robot = webots_node.robot

        # Configure the altimeter and use the robot simulation step for sampling.
        self.__timestep = int(self.__robot.getBasicTimeStep())
        self.__devices = {
            'altimeter': self.__robot.getDevice('altimeter_cube'),
        }
        self.__devices['altimeter'].enable(self.__timestep)

        # Initialize the four propeller motors used by the quadrotor.
        self.__motors = {
            'front_left':  self.__robot.getDevice('motor_propeller_front_left'),
            'front_right': self.__robot.getDevice('motor_propeller_front_right'),
            'rear_left':   self.__robot.getDevice('motor_propeller_rear_left'),
            'rear_right':  self.__robot.getDevice('motor_propeller_rear_right')
        }

        # Put the motors in velocity-control mode and apply a small startup speed.
        for motor in self.__motors.values():
            motor.setPosition(float('inf'))
            motor.setVelocity(1.0)

        sleep(1)

        # Controller gains and the initial altitude setpoint.
        self.k_vertical_thrust = 478
        self.k_vertical_offset = 0.6
        self.k_vertical_p = 30.0
        self.k_roll_p = 500.0
        self.k_pitch_p = 300.0
        self.target_altitude = 0.5

        # Latest command and sensor messages used by the control loop.
        self.__target_twist = Twist()
        self.__imu_data = Imu()
        self.__gyro_data = Imu()
        self.altimeter_msg = FloatStamped()
        self.velocity_motor_msg = JointState()

        # ROS 2 node, command/sensor subscriptions, and telemetry publishers.
        rclpy.init(args=None)
        self.__node = rclpy.create_node('thaiquiri_driver')
        self.__node.create_subscription(
            Twist, 'cmd_pos', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(
            Imu, '/drone/imu/data', self.__imu_callback, 1)
        self.__node.create_subscription(
            Imu, '/drone/cube/gyro', self.__gyro_callback, 1)
        self.__altimeter_pub = self.__node.create_publisher(
            FloatStamped, '/drone/altimeter_cube', 10)
        self.__velocity_pub = self.__node.create_publisher(
            JointState, '/drone/velocity', 10)

    def __cmd_vel_callback(self, twist):
        """Store the latest desired translational and yaw commands."""
        self.__target_twist = twist

    def __imu_callback(self, imu_msg):
        """Store the latest inertial measurement used for attitude control."""
        self.__imu_data = imu_msg

    def __gyro_callback(self, gyro_msg):
        """Store the latest angular-velocity measurement."""
        self.__gyro_data = gyro_msg

    def clamp(self, n, min_n, max_n):
        """Constrain ``n`` to the inclusive interval ``[min_n, max_n]``."""
        return max(min_n, min(n, max_n))

    def step(self):
        """Process ROS messages, update control outputs, and publish telemetry.

        The attitude and altitude errors are converted into four motor commands
        using proportional control and the quadrotor motor-mixing equations.
        """
        rclpy.spin_once(self.__node, timeout_sec=0)

        roll = self.__imu_data.orientation.x
        pitch = self.__imu_data.orientation.y
        altitude = self.__devices['altimeter'].getValue()
        roll_acceleration = self.__gyro_data.angular_velocity.x
        pitch_acceleration = self.__gyro_data.angular_velocity.y

        self.altimeter_msg.header.stamp = self.__node.get_clock().now().to_msg()
        self.altimeter_msg.header.frame_id = 'altimeter_cube'
        self.altimeter_msg.data = altitude

        self.__altimeter_pub.publish(self.altimeter_msg)

        pitch_disturbance = self.__target_twist.linear.x
        roll_disturbance = self.__target_twist.linear.y
        self.target_altitude += self.__target_twist.linear.z
        yaw_disturbance = self.__target_twist.angular.z
        self.__target_twist.linear.z = 0.0
        self.__target_twist.angular.z = self.__target_twist.angular.z * \
            0.7 if abs(self.__target_twist.angular.z) > 0 else 0

        roll_input = self.k_roll_p * \
            self.clamp(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance
        pitch_input = self.k_pitch_p * \
            self.clamp(pitch, -1.0, 1.0) + \
            pitch_acceleration + pitch_disturbance
        yaw_input = yaw_disturbance
        clamped_difference_altitude = self.clamp(
            self.target_altitude - altitude + self.k_vertical_offset, -1.0, 1.0)
        vertical_input = self.k_vertical_p * \
            pow(clamped_difference_altitude, 3.0)

        front_left_motor_input = self.k_vertical_thrust + \
            vertical_input - roll_input + pitch_input - yaw_input
        front_right_motor_input = self.k_vertical_thrust + \
            vertical_input + roll_input + pitch_input + yaw_input
        rear_left_motor_input = self.k_vertical_thrust + \
            vertical_input - roll_input - pitch_input + yaw_input
        rear_right_motor_input = self.k_vertical_thrust + \
            vertical_input + roll_input - pitch_input - yaw_input

        self.__motors['front_left'].setVelocity(-front_left_motor_input)
        self.__motors['front_right'].setVelocity(front_right_motor_input)
        self.__motors['rear_left'].setVelocity(rear_left_motor_input)
        self.__motors['rear_right'].setVelocity(-rear_right_motor_input)

        self.velocity_motor_msg.header.stamp = self.__node.get_clock().now().to_msg()
        self.velocity_motor_msg.name = [
            'front_left', 'front_right', 'rear_left', 'rear_right']
        self.velocity_motor_msg.velocity = [
            -front_left_motor_input, front_right_motor_input, rear_left_motor_input, -rear_right_motor_input]
        self.velocity_motor_msg.effort = [0.0, 0.0, 0.0, 0.0]

        self.__velocity_pub.publish(self.velocity_motor_msg)
