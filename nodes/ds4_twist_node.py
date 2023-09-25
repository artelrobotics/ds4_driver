#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from ds4_driver.msg import Status, Feedback
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
import threading
import time
from nav_msgs.msg import Odometry

class StatusToTwist(object):
        
    def __init__(self):
        self.rate = rospy.Rate(20) # rate 
        self.transmission_type = rospy.get_param("~transmission", "manual") # getting trasnmission type from param
        self._inputs = rospy.get_param("~inputs") # getting linear and angular speed params
        # Initialize variables
        self.msg_feedback = Feedback()
        self._stamped = False
        self.scale = 0.1
        self.last_state_l2 = 1
        self.last_state_r2 = 1
        self.last_state_trackpad = 0
        self.cmd_publish = True
        self.counter = 0
        self.last_led = False
        self.msg = Status()
        self._attrs = []
        self.last_msg = 0
        
        # Publishers
        self._pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.joystick_feedback = rospy.Publisher("set_feedback", Feedback, queue_size=1)
        
        #Subscribers
        rospy.Subscriber("status", Status, self.cb_status, queue_size=1)
        # rospy.Subscriber("driver/fault_flag", channel_values, self.fault_flags_callback)
        # rospy.Subscriber("encoder/odom", Odometry, self.odometry_callback)
        
        for attr in Status.__slots__:
            if attr.startswith("axis_") or attr.startswith("button_"):
                self._attrs.append(attr)

    def odometry_callback(self, msg):
        """ 
            Odometry callback function:
            if transmission type is 'auto' it works
        """
        if self.transmission_type == "auto":
            self.linear_x = abs(msg.twist.twist.linear.x)
            self.angular_z = abs(msg.twist.twist.angular.z)
            self.scale = max(self.linear_x, self.angular_z) + 0.1

    def sound_callback(self, msg):
        self.last_sound = msg.data    
    
    def fault_flags_callback(self, value):
        msg = value.value[0]
        
        if msg != 0 and self.last_msg == 0 and self.cmd_publish:
            self.set_color([1,1,0])
        
        if msg == 0 and self.last_msg != 0 and self.cmd_publish:
            self.set_color([0,1,0])
        
        self.last_msg = msg

    def set_color(self, color_list):
        """
            Setting color of joystick
            recieves color list type : list
            publish self.msg_feedback message to feedback topic
        """
        self.msg_feedback.set_rumble = False
        self.msg_feedback.set_led = True  
        self.msg_feedback.set_led_flash = True
        self.msg_feedback.led_flash_on = True
        self.msg_feedback.led_r, self.msg_feedback.led_g, self.msg_feedback.led_b = color_list
        self.joystick_feedback.publish(self.msg_feedback)

    
    def speed_up(self):
        """ In manual tranmission it increase speed """
        if self.scale < 1.1:
            self.scale += 0.1
            self.msg_feedback.set_rumble = True
            self.msg_feedback.rumble_small = 1
            self.msg_feedback.rumble_duration = 0.2
            self.joystick_feedback.publish(self.msg_feedback)
    
    def speed_down(self):
        """ In manual tranmission it decrease speed """
        if self.scale >= 0.2:
                self.scale -= 0.1
                self.msg_feedback.set_rumble = True
                self.msg_feedback.rumble_small = 1
                self.msg_feedback.rumble_duration = 0.2
                self.joystick_feedback.publish(self.msg_feedback)
    
    def cb_status(self, msg):
        """
            Joystick status callback function
            param msg:
            type msg: Status
            publish: /cmd_vel
        """
        twist = Twist()
        
        if self.cmd_publish and not self.last_led:            
            self.set_color([0,1,0])
        
        if self.cmd_publish and not self.last_led:            
            self.set_color([1,1,0])
                   
        if not self.cmd_publish and self.last_led:
            self.set_color([1,0,0])
        
        input_vals = {}
        for attr in self._attrs:
            input_vals[attr] = getattr(msg, attr)
        
            
        if msg.button_trackpad and not self.last_state_trackpad:
            self.counter += 1
        
        if self.transmission_type == "imu":
            twist.linear.x = msg.imu.linear_acceleration.z * 0.1
            twist.angular.z = msg.imu.linear_acceleration.x * 0.1

        if self.transmission_type == "manual":
            if msg.button_r2 and not self.last_state_r2:     
                self.speed_up()
            if msg.button_l2 and not self.last_state_l2:
                self.speed_down()
            
            self.last_state_l2 = msg.button_l2
            self.last_state_r2 = msg.button_r2    

        self.last_state_trackpad = msg.button_trackpad
        self.last_led = self.cmd_publish
        
        if self.transmission_type != "imu":
            for vel_type in self._inputs:
                vel_vec = getattr(twist, vel_type)
                for k, expr in self._inputs[vel_type].items():
                    val = eval(expr, {}, input_vals)
                    if k == "z":
                        setattr(vel_vec, k, 2 * self.scale * val)
                    else:
                        setattr(vel_vec, k, self.scale * val)
            
        if self.counter % 2 == 0:
            self._pub.publish(twist)
            self.cmd_publish = True
        
        if self.counter % 2 == 1 and self.cmd_publish:
            self.cmd_publish = False
            twist.linear.x = 0
            twist.angular.z = 0
            self._pub.publish(twist)

        self.rate.sleep()
                
def main():
    rospy.init_node("ds4_twist")
    StatusToTwist()
    rospy.spin()

if __name__ == "__main__":
    main()