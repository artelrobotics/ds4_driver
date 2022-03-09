#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from ds4_driver.msg import Status


class StatusToTwist(object):
    def __init__(self):
        self._stamped = rospy.get_param("~stamped", False)
        if self._stamped:
            self._cls = TwistStamped
            self._frame_id = rospy.get_param("~frame_id", "base_link")
        else:
            self._cls = Twist
        self._inputs = rospy.get_param("~inputs")
        # self._scales = rospy.get_param("~scales")
        self.scale = 0.1
        self.last_state_l2 = 1
        self.last_state_r2 = 1
        self.msg = Status()
        self._attrs = []
        for attr in Status.__slots__:
            if attr.startswith("axis_") or attr.startswith("button_"):
                self._attrs.append(attr)

        self._pub = rospy.Publisher("driver/cmd_vel", self._cls, queue_size=1)
        rospy.Subscriber("status", Status, self.cb_status, queue_size=1)

    def cb_status(self, msg):
        """
        :param msg:
        :type msg: Status
        :return:
        """
        self.msg = msg
        input_vals = {}
        for attr in self._attrs:
            input_vals[attr] = getattr(msg, attr)
        
        if msg.button_r2 and not self.last_state_r2:
            if self.scale < 1.1:
                self.scale += 0.1
            
        if msg.button_l2 == 1 and self.last_state_l2 == 0:
            if self.scale > 0.2:
                self.scale -= 0.1
        
        self.last_state_l2 = msg.button_l2
        self.last_state_r2 = msg.button_r2
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
                setattr(vel_vec, k, self.scale * val)

        self._pub.publish(to_pub)

def main():
    rospy.init_node("ds4_twist")

    StatusToTwist()

    rospy.spin()


if __name__ == "__main__":
    main()
