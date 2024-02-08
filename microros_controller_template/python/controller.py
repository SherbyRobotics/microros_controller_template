

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy

MODE1, MODE2, MODE3, MODE4 = range(0, 4)


class Template_node(Node): 
    def __init__(self):
        super().__init__("reab_controller") 

        # publisher 
        self.pub_cmd = self.create_publisher(Float32MultiArray, "cmd", 1)

        #subscriber
        self.sub_joy = self.create_subscription(Joy, "joy", self.read_joy, 1) 
        self.sub_sensors = self.create_subscription(Float32MultiArray, "sensors", self.read_sensors, 1) 

        # Timer
        self.dt = 0.05  # 50 Hz
        self.timer = self.create_timer(self.dt, self.timed_controller) 

        self.high_level_mode = 0.0
        self.high_level_submode = 0.0
        self.high_level_cmd = 0.0
        self.mode = 0.0



    def timed_controller(self):

        if self.high_level_mode == 1.0 and self.high_level_submode == 1.0 :
            self.mode = MODE1 

        if self.high_level_mode == 1.0 and self.high_level_submode == 2.0 : 
            self.mode = MODE2

        if self.high_level_mode == 2.0 and self.high_level_submode == 1.0 : 
            self.mode = MODE3
       
        if self.high_level_mode == 2.0 and self.high_level_submode == 2.0 : 
            self.mode = MODE4

        self.send_cmd_on_publisher()


    def read_joy(self, joy_msg):
        """
        This function reads the /joy topic and changes the mode based on the buttons pressed
        """

        if True: # It is possible to add a condition like a deadman switch

            # If button back is active
            if joy_msg.buttons[6]:
                self.high_level_mode = 1.0

            # If button home logitech is active
            elif joy_msg.buttons[8]:
                pass

            # If button start is active
            elif joy_msg.buttons[7]:
                self.high_level_mode = 2.0

            # If right joystick is pressed
            elif joy_msg.buttons[10]:
                pass

            # If left joystick is pressed
            elif joy_msg.buttons[9]:
                pass

            # If button A is active
            elif joy_msg.buttons[0]:
                self.high_level_submode = 1.0

            # If button B is active
            elif joy_msg.buttons[1]:
                self.high_level_submode = 2.0

            # If button X is active
            elif joy_msg.buttons[2]:
                pass

            # If bottom Y is active
            elif joy_msg.buttons[3]:
                pass

            # If bottom LB is active
            elif joy_msg.buttons[4]:
                pass
            
            # If bottom RB is active
            elif joy_msg.buttons[5]:
                pass

            # If left right d-pad
            elif joy_msg.axes[6]:
                pass

            # If up down d-pad
            elif joy_msg.axes[7]:
                pass


        else:
            self.high_level_mode = 0.0
            self.high_level_submode = 0.0
        
        self.high_level_cmd = joy_msg.axes[4]  # Up-down right joystick 


    def read_sensors(self, msg):

        feedback = int( msg.data[0] )

        if feedback ==  MODE1:
            self.get_logger().info("tacos")
        elif feedback ==  MODE2:
            self.get_logger().info("pizza")
        elif feedback ==  MODE3:
            self.get_logger().info("hot-dog")
        elif feedback ==  MODE4:
            self.get_logger().info("hamburger")




    def send_cmd_on_publisher(self):

        """
        This function fills and publishes the "/cmd" message
        """

        # Init msg
        cmd_msg = Float32MultiArray()
        data    =  [0.0] * 3

        #Msgard            
        data[0] = float(self.mode)
        data[1] = float(self.high_level_mode)
        data[2] = float(self.high_level_submode)

        cmd_msg.data = data # affectation du tableau data au "cmd_msgs" du topic "/cmd"

        # Publish cmd msg
        self.pub_cmd.publish(cmd_msg)


#### MAIN LOOP ###############################################################################################################

def main(args=None):
    rclpy.init(args=args)
    node = Template_node() 
    node.get_logger().info("hello ros")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()