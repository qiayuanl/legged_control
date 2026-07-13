#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from ocs2_msgs.msg import mpc_observation


def mode_to_contacts(mode):
    mode = int(mode)

    # Contact order: [LF, LH, RF, RH]
    return [
        1.0 if (mode & 1) else 0.0,   # LF
        1.0 if (mode & 2) else 0.0,   # LH
        1.0 if (mode & 4) else 0.0,   # RF
        1.0 if (mode & 8) else 0.0,   # RH
    ]


class ModeToContacts:
    def __init__(self):
        rospy.init_node("mode_to_contacts", anonymous=False)

        self.pub = rospy.Publisher(
            "/isaac/contacts",
            Float64MultiArray,
            queue_size=10,
        )

        self.sub = rospy.Subscriber(
            "/legged_robot_mpc_observation",
            mpc_observation,
            self.observation_cb,
            queue_size=10,
        )

        self.last_mode = None

        rospy.loginfo("[mode_to_contacts] Started")
        rospy.loginfo("[mode_to_contacts] Publishing /isaac/contacts from MPC mode")

    def observation_cb(self, msg):
        mode = int(msg.mode)
        contacts = mode_to_contacts(mode)

        out = Float64MultiArray()
        out.data = contacts
        self.pub.publish(out)

        if mode != self.last_mode:
            rospy.loginfo("[mode_to_contacts] mode=%d -> contacts=%s", mode, contacts)
            self.last_mode = mode


if __name__ == "__main__":
    ModeToContacts()
    rospy.spin()
