#!/usr/bin/env python

"""
Simple Baxter Control Script
This script demonstrates basic Baxter robot control
"""

import rospy
import baxter_interface
from baxter_interface import Limb


def main():
    """Main function to control Baxter robot"""
    print("Initializing Baxter control node...")
    rospy.init_node('simple_baxter_control')
    
    # Enable the robot
    rs = baxter_interface.RobotEnable()
    print("Enabling robot...")
    rs.enable()
    
    # Create limb instances
    left_arm = Limb('left')
    right_arm = Limb('right')
    
    print("Robot enabled. Current joint angles:")
    print("Left arm:", left_arm.joint_angles())
    print("Right arm:", right_arm.joint_angles())
    
    # Simple wave motion
    print("Performing simple wave motion...")
    
    # Move right arm to wave position
    wave_angles = {
        'right_s0': -0.5,
        'right_s1': -1.0,
        'right_e0': 1.5,
        'right_e1': 1.5,
        'right_w0': 0.0,
        'right_w1': 0.5,
        'right_w2': 0.0
    }
    
    right_arm.move_to_joint_positions(wave_angles)
    
    # Wave motion
    for i in range(3):
        right_arm.set_joint_position('right_w2', 1.0)
        rospy.sleep(0.5)
        right_arm.set_joint_position('right_w2', -1.0)
        rospy.sleep(0.5)
    
    # Return to neutral position
    print("Returning to neutral position...")
    right_arm.move_to_neutral()
    left_arm.move_to_neutral()
    
    print("Demo complete!")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass