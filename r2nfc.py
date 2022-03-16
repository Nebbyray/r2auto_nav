import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import RPi.GPIO as GPIO
from pn532 import *

# constants
rotatechange = 0.1
speedchange = 0.05
message_sent = 'Not Detected'

# Set up NFC reader
#pn532 = PN532_SPI(debug=False, reset=20, cs=4)
pn532 = PN532_I2C(debug=False, reset=20, req=16)
#pn532 = PN532_UART(debug=False, reset=20)

ic, ver, rev, support = pn532.get_firmware_version()
print('Found PN532 with firmware version: {0}.{1}'.format(ver, rev))

# Configure PN532 to communicate with MiFare cards
pn532.SAM_configuration()


class NFC(Node):
    def __init__(self):
        super().__init__('nfc')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.nfc_pub = self.create_publisher(String, 'nfc_status', 10)
        #self.get_logger().info('Created NFC publisher')
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        global message_sent
        msg = String()
        msg.data = message_sent
        self.nfc_pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def stopbot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)


    def find_nfc(self):
        global message_sent
        nfc_found = False

        while not nfc_found:
            detected = False
            
            # Check if a card is available to read
            uid = pn532.read_passive_target(timeout=0.5)
            print('.', end="")
            if uid is not None:
                detected = True
                nfc_found = True

            if detected == True:
                message_sent = 'DetectedNFC'
                self.timer_callback()
                print('Found card with UID', [hex(i) for i in uid])
                time.sleep(1)

        # If target is found, stop movement
        self.stopbot()
        return True
                

    def detecting(self):
        global message_sent

        # find the nfc
        self.find_nfc()
        
        time.sleep(30)

        # -------------- #
        # Do the cleanup #
        # -------------- #
        # Send message that the robot has been loaded
        
        message_sent = 'DoneLoading'
        self.timer_callback()

        # Cleanup all GPIO
        GPIO.cleanup()
        self.get_logger().info("Cleaned up GPIO")
    

def main(args=None):
    rclpy.init(args=args)
    
    nfc = NFC()
    nfc.detecting()
    
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    NFC_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
