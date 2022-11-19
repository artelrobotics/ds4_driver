#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from ds4_driver.msg import Status, Feedback
from nav_msgs.msg import Odometry

class StatusToTwist(object):
    def __init__(self):
        self._stamped = rospy.get_param("~stamped", False)
        if self._stamped:
            self._cls = TwistStamped
            self._frame_id = rospy.get_param("~frame_id", "base_link")
        else:
            self._cls = Twist
        self._inputs = rospy.get_param("~inputs")

        self.last_state_l1 = 1
        self.last_state_r1 = 1
        self.msg = Status()
        self._attrs = []
        for attr in Status.__slots__:
            if attr.startswith("axis_") or attr.startswith("button_"):
                self._attrs.append(attr)

        self._pub = rospy.Publisher("cmd_vel", self._cls, queue_size=1)
        rospy.Subscriber("status", Status, self.cb_status, queue_size=1)
        rospy.Subscriber("odometry", Odometry, self.odometry_cb, queue_size=1)
        self.vibrator_toy = rospy.Publisher("set_feedback", Feedback, queue_size=1)

    def odometry_cb(self, msg):
        self.scale = max(abs(msg.twist.twist.linear.x), abs(msg.twist.twist.angular.z)) + 0.1
 

    def cb_status(self, msg):
        """
        :param msg:
        :type msg: Status
        :return:
        """
        msg_feedback = Feedback()
        input_vals = {}
        for attr in self._attrs:
            input_vals[attr] = getattr(msg, attr)

        # if msg.button_r2 and not self.last_state_r2:
        #     if self.scale < 2.3:
        #         self.scale += 0.1
        #         msg_feedback.set_rumble = True
        #         msg_feedback.rumble_small = 1
        #         msg_feedback.rumble_duration = 0.2
        #         self.vibrator_toy.publish(msg_feedback) 
        
        # if msg.button_l2 == 1 and self.last_state_l2 == 0:
        #     if self.scale > 0.2:
        #         self.scale -= 0.1
        #         msg_feedback.set_rumble = True
        #         msg_feedback.rumble_small = 1
        #         msg_feedback.rumble_duration = 0.2
        #         self.vibrator_toy.publish(msg_feedback)
        
        # self.last_state_l2 = msg.button_l2
        # self.last_state_r2 = msg.button_r2
        # self.last_state_l1 = msg.button_l1
        # self.last_state_r1 = msg.button_r1
        

        to_pub = self._cls()
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
                    setattr(vel_vec, k, 3 * self.scale * val)
                else:
                    setattr(vel_vec, k, self.scale * val)

        self._pub.publish(to_pub)


def main():
    rospy.init_node("ds4_twist")

    StatusToTwist()

    rospy.spin()


if __name__ == "__main__":
    main()
