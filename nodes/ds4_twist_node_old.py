#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from ds4_driver.msg import Status, Feedback
from roboteq_motor_controller_driver.srv import emergency_stop_srv
from std_srvs.srv import SetBool
from camel_robot.srv import light, sound
from std_msgs.msg import Bool
from roboteq_motor_controller_driver.msg import channel_values
import threading
import time
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class StatusToTwist(object):
        
    def __init__(self):
        self.rate = rospy.Rate(20)
        self._stamped = rospy.get_param("~stamped", False)
        if self._stamped:
            self._cls = TwistStamped
            self._frame_id = rospy.get_param("~frame_id", "base_link")
        else:
            self._cls = Twist
        self._inputs = rospy.get_param("~inputs")
        self.scale = 0.1
        self.last_state_l2 = 1
        self.last_state_r2 = 1
        self.last_state_l1 = 1
        self.last_state_r1 = 1
        self.emergancy_status = 0
        self.last_emergancy_status = 0
        self.last_state_trackpad = 0
        self.led = True
        self.counter = 0
        self.last_led = False
        self.red = 0.0
        self.green = 1.0
        self.blue = 0.0
        self.msg = Status()
        self._attrs = []
        
        for attr in Status.__slots__:
            if attr.startswith("axis_") or attr.startswith("button_"):
                self._attrs.append(attr)

        self._pub = rospy.Publisher("cmd_vel", self._cls, queue_size=1)
        self.vibrator_toy = rospy.Publisher("set_feedback", Feedback, queue_size=1)
        # self.obstacle_avoidance = rospy.Publisher("obstacle_avoidance", Bool, queue_size=1)
        self.emergancy_stop = rospy.ServiceProxy("driver/emergency_stop_service", emergency_stop_srv)
        # self.hook_service = rospy.ServiceProxy("common/hooks_ctrl", SetBool)
        # self.light_service = rospy.ServiceProxy("common/Light_server", light)
        # self.sound_service = rospy.ServiceProxy("sound_server", sound)
        # self.docking_service = rospy.ServiceProxy("Docking_server", light)
        
        rospy.Subscriber("status", Status, self.cb_status, queue_size=1)
        rospy.Subscriber("driver/fault_flag", channel_values, self.fault_flags_callback)
        # rospy.Subscriber('/camel_amr_1000_001/sound_state', String, self.sound_callback)
    


    def sound_callback(self, msg):
        self.last_sound = msg.data    
    
    def fault_flags_callback(self, value):
        self.emergancy_status = value.value[0]
        return value.value[0]

        
    def cb_status(self, msg):
        """
        :param msg:
        :type msg: Status
        :return:
        """
        msg_feedback = Feedback()
        self.msg = msg
        msg_feedback.set_led = True
        msg_feedback.set_led_flash = True
        input_vals = {}
        
        if self.led and not self.last_led and self.counter % 2 == 0 and self.emergancy_status == 0:            
            self.red = 0.0
            self.green = 1.0
            self.blue = 0.0    
            msg_feedback.led_r = self.red
            msg_feedback.led_g = self.green
            msg_feedback.led_b = self.blue
            self.vibrator_toy.publish(msg_feedback)
            
        if not self.led and self.last_led or self.counter % 2 == 1:
            self.red = 1.0
            self.green = 0.0
            self.blue = 0.0    
            msg_feedback.led_r = self.red
            msg_feedback.led_g = self.green
            msg_feedback.led_b = self.blue
            self.vibrator_toy.publish(msg_feedback)
        
        if self.emergancy_status == 0 and self.counter % 2 == 0 and not msg.button_r2 and not msg.button_l2: 
            self.red = 0.0
            self.green = 1.0
            self.blue = 0.0    
            msg_feedback.led_r = self.red
            msg_feedback.led_g = self.green
            msg_feedback.led_b = self.blue
            self.vibrator_toy.publish(msg_feedback)
        
        if self.emergancy_status != 0 and self.counter % 2 == 0: 
            self.red = 1.0
            self.green = 1.0
            self.blue = 0.0    
            msg_feedback.led_r = self.red
            msg_feedback.led_g = self.green
            msg_feedback.led_b = self.blue
            self.vibrator_toy.publish(msg_feedback)
        

        for attr in self._attrs:
            input_vals[attr] = getattr(msg, attr)
        
        # if msg.button_dpad_up:
        #     #try:
        #     self.hook_service(True)
        #     time.sleep(0.2)
        #     #except rospy.ServiceException as e:
        #     #    rospy.logerr(e)
        
        # if msg.button_dpad_down:
        #     #try:
        #     self.hook_service(False)
        #     time.sleep(0.2)
        #     #except rospy.ServiceException as e:
        #     #    rospy.logerr(e)

        if msg.button_options:
            try:
                self.emergancy_stop(True)
                time.sleep(0.1)
            except rospy.ServiceException as e:
                rospy.logerr(e)
                    
        if msg.button_share:
            try:
                self.emergancy_stop(False)
                time.sleep(0.1)
            except rospy.ServiceException as e:
                rospy.logerr(e)

        

        if msg.button_r2 and not self.last_state_r2:
            if self.scale < 1.1:
                self.scale += 0.1
                msg_feedback.led_r = self.red
                msg_feedback.led_g = self.green
                msg_feedback.led_b = self.blue
                msg_feedback.set_rumble = True
                msg_feedback.rumble_small = 1
                msg_feedback.rumble_duration = 0.2
                self.vibrator_toy.publish(msg_feedback)        
                
        if msg.button_l2 == 1 and self.last_state_l2 == 0:
            if self.scale > 0.2:
                self.scale -= 0.1
                msg_feedback.led_r = self.red
                msg_feedback.led_g = self.green
                msg_feedback.led_b = self.blue
                msg_feedback.set_rumble = True
                msg_feedback.rumble_small = 1
                msg_feedback.rumble_duration = 0.2
                self.vibrator_toy.publish(msg_feedback)
        
        # if msg.button_l1 == 1 and self.last_state_l1 == 0:
        #     self.thread_2 = threading.Thread(target=self.light_up)
        #     self.thread_2.start()
            
        # if msg.button_r1 == 1 and self.last_state_r1 == 0:
        #     self.light_service('light_type_5')
        #     time.sleep(0.1)
        #     self.light_service('light_type_2')
        #     time.sleep(0.1)
       
        if msg.button_trackpad and not self.last_state_trackpad:
            self.counter += 1
        
        self.last_led = self.led
        self.last_state_l2 = msg.button_l2
        self.last_state_r2 = msg.button_r2
        self.last_state_l1 = msg.button_l1
        self.last_state_r1 = msg.button_r1
        self.last_state_trackpad = msg.button_trackpad
        self.last_emergancy_status = self.emergancy_status
        
        to_pub = self._cls()
        
        # if msg.button_triangle:
        #     self.thread_1 = threading.Thread(target=self.docking)
        #     self.thread_1.start()
        #     # self.docking_service(True)
        
        # if msg.button_cross:
        #     self.thread_2 = threading.Thread(target=self.undocking)
        #     self.thread_2.start()
        #     # self.docking_service(True)
        
        # if msg.button_square:
        #     self.thread_2 = threading.Thread(target=self.charging)
        #     self.thread_2.start()
        
        # if msg.button_circle:
        #     self.thread_2 = threading.Thread(target=self.discharging)
        #     self.thread_2.start()
        

        # if msg.button_r1 and msg.button_l1:
        #     self.sound_service("sound_stop")
            

        if self._stamped:
            to_pub.header.stamp = rospy.Time.now()
            to_pub.header.frame_id = self._frame_id
            twist = to_pub.twist
        else:
            twist = to_pub

        for vel_type in self._inputs:
            vel_vec = getattr(twist, vel_type)
            for k, expr in self._inputs[vel_type].items():
                # scale = self._scales[vel_type].get(k, 1.0)
                val = eval(expr, {}, input_vals)
                if k == "z":
                    setattr(vel_vec, k, 2 * self.scale * val)
                else:
                    setattr(vel_vec, k, self.scale * val)
        
        if self.counter % 2 == 0:
            self._pub.publish(to_pub)
            self.led = True
        
        else:
            self.led = False
        self.rate.sleep()
    # def docking(self):
    #     try:
    #         resp = self.docking_service("loading")
    #     except Exception as e:
    #         pass

    # def undocking(self):
    #     try:
    #         resp = self.docking_service("unloading")
    #     except Exception as e:
    #         pass
    
    # def charging(self):
    #     try:
    #         resp = self.docking_service("charging")
    #     except Exception as e:
    #         pass

    # def discharging(self):
    #     try:
    #         resp = self.docking_service("discharging")
    #     except Exception as e:
    #         pass
    
    # def light_up(self):
    #     try:
    #         resp = self.sound_service('beep')
    #         time.sleep(0.2)
    #         resp = self.light_service('light_type_6')
    #         time.sleep(0.3)
    #         resp = self.light_service('light_type_2')
    #         time.sleep(0.2)
    #         resp = self.sound_service(self.last_sound)
    #         time.sleep(1)
    #     except Exception as e:
    #         pass


            
def main():
    rospy.init_node("ds4_twist")
    rospy.Rate(10)
    StatusToTwist()

    rospy.spin()


if __name__ == "__main__":
    main()
