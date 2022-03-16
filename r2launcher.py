import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Bool, String
import time
import busio
import board
import adafruit_amg88xx
import RPi.GPIO as GPIO

# constants
rotatechange = 0.1
speedchange = 0.05
detecting_threshold = 32.0
firing_threshold = 35.0
servo_pin = 14
MotorL_pin = 
MotorR_pin = 


message_sent = 'Not Detected'

# Set up Thermal Camera
i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c)


class ThermalCamera(Node):
    def __init__(self):
        super().__init__('thermalcamera')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # Set up publisher 'targeting_status' to communicate with wallfollower
        self.publisher_targeting = self.create_publisher(
            String, 'targeting_status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    # targeting_status callback function to stop wallfollower logic when target is detected
    def timer_callback(self):
        global message_sent
        msg = String()
        msg.data = message_sent
        self.publisher_targeting.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def stopbot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def find_target(self):

        # See if target found
        global message_sent
        target_found = False

        # -------------------------------------------------------------------------- #
        # While target has not been found, run this code to check if target is found #
        # -------------------------------------------------------------------------- #
        while not target_found:
            detected = False

            for row in amg.pixels:
                print('[', end=" ")
                for temp in row:
                    if temp > detecting_threshold:
                        detected = True
                        target_found = True
                    print("{0:.1f}".format(temp), end=" ")
                print("]")
                print("\n")

            if detected == True:
                # Communicate with wallfollower to stop working
                message_sent = 'DetectedTarget'
                self.timer_callback()
                print(" ")
                print("DETECTED!!")
                print("]")
                print("\n")
                time.sleep(1)

        # If target is found, stop movement
        self.stopbot()

        return True

    def centre_target(self):
        # ----------------------------------------------------------- #
        # Adjust the servo and robot until high temp is in the centre #
        # ----------------------------------------------------------- #

        # Centre the target in the robot's vision
        GPIO.setmode(GPIO.BCM)
        horizontally_centered = False
        vertically_centered = False
        centered = False

        while not centered:
            screen = amg.pixels
            max_row = 0
            max_column = 0
            max_value = 0.0
            for row in range(len(screen)):
                for column in range(len(screen[row])):
                    current_value = screen[row][column]
                    if current_value > max_value:
                        max_row = row
                        max_column = column
                        max_value = current_value

            if not horizontally_centered:
                # centre max value between row 3 and 4
                if max_column < 3:
                    # spin it anti-clockwise
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = rotatechange
                    time.sleep(1)
                    self.publisher_.publish(twist)
                    time.sleep(1)
                elif max_column > 4:
                    # spin it clockwise
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = -1 * rotatechange
                    time.sleep(1)
                    self.publisher_.publish(twist)
                    time.sleep(1)
                else:
                    horizontally_centered = True

                self.stopbot()

            if horizontally_centered and not vertically_centered:
                # centre max value between row 3 and 4
                if max_row < 3:
                    # shift the servo up by 5 degrees (limit:0)
                    try:
                        servo_pin = 4

                        GPIO.setup(servo_pin, GPIO.OUT)

                        p = GPIO.PWM(servo_pin, 50)

                        p.start(2.5)

                        degree = 0

                        servo_value = degree/90 * 5 + 2.5
                        p.ChangeDutyCle(servo_value)
                        time.sleep(1)
                    except:
                        p.stop()
                        GPIO.cleanup()
                    finally:
                        p.stop()
                        GPIO.cleanup()

                elif max_row > 4:
                    # shift the servo down by 5 degrees (limit: 20)
                    try:
                        servo_pin = 4

                        GPIO.setup(servo_pin, GPIO.OUT)

                        p = GPIO.PWM(servo_pin, 50)

                        p.start(2.5)

                        degree = 20

                        servo_value = degree/90 * 5 + 2.5
                        p.ChangeDutyCle(servo_value)
                        time.sleep(1)
                    except:
                        p.stop()
                        GPIO.cleanup()
                    finally:
                        p.stop()
                        GPIO.cleanup()

                else:
                    vertically_centered = True

            if horizontally_centered and vertically_centered:
                centered = True

            return True

    def move_to_target(self):
        # move to the object in increments
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        time.sleep(1)
        self.stopbot()

    def firing_time(self):
        screen = amg.pixels
        for row in [3, 4]:
            for column in [3, 4]:
                if screen[row][column] > firing_threshold:
                    return True

    def targetting(self):
        global message_sent

        # find the target
        self.find_target()

        # centre the target
        self.centre_target()

        # --------------------------------------------------- #
        # Now it is centered, start moving towards the target #
        # --------------------------------------------------- #

        while not self.firing_time():
            self.move_to_target()
            self.centre_target()

        # ----------------------------- #
        # Now the bot can fire the ball #
        # ----------------------------- #

        GPIO.setmode(GPIO.BCM)
        
        # Setup DC motors
        GPIO.setup(MotorL_pin, GPIO.OUT)
        GPIO.setup(MotorR_pin, GPIO.OUT)
        self.get_logger().info("Setup the DC")

        # Setup servo motor
        GPIO.setup(servo_pin, GPIO.OUT)
        p = GPIO.PWM(servo_pin, 50)
        p.start(0)
        
        time.sleep(5)

        # Spin Backwards Continuously
        GPIO.output(MotorL_pin, True)
        GPIO.output(MotorR_pin, True)
        self.get_logger().info("Started the DC Motor")
        
        # Move the servo arm lock
        p.ChangeDutyCycle(75)
        GPIO.output(servo_pin, True)
        
        # Wait for all balls to be shot
        time.sleep(10)
        

        # -------------- #
        # Do the cleanup #
        # -------------- #
        # Send message that the target has finished shooting
        message_sent = 'DoneShooting'
        self.timer_callback()

        # Stop the DC Motor
        GPIO.output(MotorL_pin, False)
        GPIO.output(MotorR_pin, False)
        self.get_logger().info("Stopped the DC Motor")
        p.stop()

        # Cleanup all GPIO
        GPIO.cleanup()
        self.get_logger().info("Cleaned up GPIO")


def main(args=None):
    rclpy.init(args=args)

    thermalcamera = ThermalCamera()
    thermalcamera.targetting()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    thermalcamera.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
