import eua_control
import rclpy
import threading


def main(args=None):
    rclpy.init(args=args)
    controller = eua_control.EUAController()
    thread = threading.Thread(target=controller.run)
    thread.start()
    rclpy.spin(controller)
    thread.join()
    rclpy.shutdown()
