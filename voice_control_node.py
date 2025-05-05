import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import speech_recognition as sr

class VoiceControl(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(5.0, self.listen_command)

    def listen_command(self):
        recognizer = sr.Recognizer()
        with sr.Microphone() as source:
            self.get_logger().info("Say a command...")
            audio = recognizer.listen(source)

            try:
                command = recognizer.recognize_google(audio).lower()
                self.get_logger().info(f"You said: {command}")
                self.send_velocity(command)
            except Exception as e:
                self.get_logger().warn(f"Error recognizing voice: {e}")

    def send_velocity(self, command):
        msg = Twist()

        if "forward" in command:
            msg.linear.x = 0.5
        elif "backward" in command:
            msg.linear.x = -0.5
        elif "left" in command:
            msg.angular.z = 0.5
        elif "right" in command:
            msg.angular.z = -0.5
        elif "stop" in command:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:
            self.get_logger().info("Command not recognized")

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
