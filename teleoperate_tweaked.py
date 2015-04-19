#!/usr/bin/env python

"""
Human Robot Interation, Design Project II
"""
import argparse
import sys

import rospy
import tf2_ros
import math
import numpy as np

import baxter_interface
from baxter_interface import CHECK_VERSION

BASE_FRAME = 'camera_depth_frame'
FRAMES = [
        'torso',
        'left_shoulder',
        'left_elbow',
        'left_hand',
        'right_shoulder',
        'right_elbow',
        'right_hand',
        ]

TEST_JOINT_ANGLES = dict()
TEST_JOINT_ANGLES['left'] = dict()
TEST_JOINT_ANGLES['right'] = dict()
TEST_JOINT_ANGLES['left']['left_s0'] = 0.0
TEST_JOINT_ANGLES['left']['left_s1'] = 0.0 
TEST_JOINT_ANGLES['left']['left_e0'] = 0.0
TEST_JOINT_ANGLES['left']['left_e1'] = 0.0 
TEST_JOINT_ANGLES['left']['left_w0'] = 0.0
TEST_JOINT_ANGLES['left']['left_w1'] = 0.0
TEST_JOINT_ANGLES['left']['left_w2'] = 0.0
TEST_JOINT_ANGLES['right']['right_s0'] = 0.0 
TEST_JOINT_ANGLES['right']['right_s1'] = 0.0
TEST_JOINT_ANGLES['right']['right_e0'] = 0.0
TEST_JOINT_ANGLES['right']['right_e1'] = math.pi 
TEST_JOINT_ANGLES['right']['right_w0'] = 0.0
TEST_JOINT_ANGLES['right']['right_w1'] = 0.0
TEST_JOINT_ANGLES['right']['right_w2'] = 0.0


def get_joint_angles(user, tfBuffer, test, mirrored):
    """

    @param line: the line described in a list to process
    @param names: joint name keys
    """
    joint_angles = dict()
    joint_angles['left'] = dict()
    joint_angles['right'] = dict()

    frames = get_frame_positions(user, tfBuffer)
    if frames is None and not test:
        # if there's a problem with tracking, don't move
        return None

    if test:
        # use the hardcoded angles for debugging
        joint_angles = TEST_JOINT_ANGLES
    else:
        # do the math to find joint angles!
        reh = frames['right_hand'] - frames['right_elbow']
        res = frames['right_shoulder'] - frames['right_elbow']
        leh = frames['left_hand'] - frames['left_elbow']
        les = frames['left_shoulder'] - frames['left_elbow']
        rse = np.negative(res)
        lse = np.negative(les)
        
        # find down vector and normals
        nt = np.cross(frames['right_shoulder'] - frames['torso'], frames['left_shoulder'] - frames['torso'])
        d = np.cross(nt, frames['right_shoulder'] - frames['left_shoulder'])
        rns = np.cross(d, rse) 
        lns = np.cross(d, lse)
        lne = np.cross(leh, les)
        rne = np.cross(reh, res)
        
        # normalize vectors
        reh = reh / np.linalg.norm(reh)
        res = res / np.linalg.norm(res)
        leh = leh / np.linalg.norm(leh)
        les = les / np.linalg.norm(les)
        rse = rse / np.linalg.norm(rse)
        lse = lse / np.linalg.norm(lse)
        nt = nt / np.linalg.norm(nt) 
        d = d / np.linalg.norm(d)
        rns = rns / np.linalg.norm(rns)
        lns = lns / np.linalg.norm(lns)
        lne = lne / np.linalg.norm(lne)
        rne = rne / np.linalg.norm(rne)

        # do the math to find joint angles
        joint_angles['left']['left_s0'] = np.arccos(np.dot(nt,lns)) - math.pi/5.0 # was + 0.0
        joint_angles['left']['left_s1'] = np.arccos(np.dot(d, lse)) - math.pi/2.0
        joint_angles['left']['left_e0'] = np.arccos(np.dot(nt, lne))
        joint_angles['left']['left_e1'] = math.pi - np.arccos(np.dot(leh, les))
        joint_angles['left']['left_w0'] = 0.0
        joint_angles['left']['left_w1'] = 0.0
        joint_angles['left']['left_w2'] = 0.0
        joint_angles['right']['right_s0'] = np.arccos(np.dot(nt,rns)) - math.pi*7.0/8.0 # was - pi
        joint_angles['right']['right_s1'] = np.arccos(np.dot(d, rse)) - math.pi/2.0
        joint_angles['right']['right_e0'] = np.arccos(np.dot(nt, rne)) - math.pi
        joint_angles['right']['right_e1'] = math.pi - np.arccos(np.dot(reh, res))
        joint_angles['right']['right_w0'] = 0.0
        joint_angles['right']['right_w1'] = 0.0
        joint_angles['right']['right_w2'] = 0.0

    if mirrored:
        return joint_angles
    else:
        # this is the default option, where the teleoperator's
        # left/right arm corresponds to the robot's left/right arm
        unmirrored = dict()
        unmirrored['left'] = dict()
        unmirrored['right'] = dict()
        unmirrored['left']['left_s0'] = joint_angles['right']['right_s0']
        unmirrored['left']['left_s1'] = joint_angles['right']['right_s1']
        unmirrored['left']['left_e0'] = joint_angles['right']['right_e0']
        unmirrored['left']['left_e1'] = joint_angles['right']['right_e1']
        unmirrored['left']['left_w0'] = 0.0 
        unmirrored['left']['left_w1'] = 0.0
        unmirrored['left']['left_w2'] = 0.0
        unmirrored['right']['right_s0'] = joint_angles['left']['left_s0']
        unmirrored['right']['right_s1'] = joint_angles['left']['left_s1']
        unmirrored['right']['right_e0'] = joint_angles['left']['left_e0']
        unmirrored['right']['right_e1'] = joint_angles['left']['left_e1']
        unmirrored['right']['right_w0'] = 0.0
        unmirrored['right']['right_w1'] = 0.0
        unmirrored['right']['right_w2'] = 0.0
        return unmirrored

def get_frame_positions(user, tfBuffer):
    frame_positions = dict()
    try:
        for frame in FRAMES:
            transformation= tfBuffer.lookup_transform(BASE_FRAME, "%s_%d" % (frame, user), rospy.Time())
            translation = transformation.transform.translation
            pos = np.array([translation.x, translation.y, translation.z])
            frame_positions[frame] = pos
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
        print "Problem with kinect tracking."
        print e
        return None
    return frame_positions

def teleoperate(rate, user, test, mirrored):
    """
    Teleoperates the robot based on tf2 frames.

    @param rate: rate at which to sample joint positions in ms

    """
    rate = rospy.Rate(rate)
    # TODO: make these attributes of a class
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')

    while not rospy.is_shutdown():
        rate.sleep()
        joint_angles = get_joint_angles(user, tfBuffer, test, mirrored)
        print joint_angles
        if joint_angles is not None:
            left.set_joint_positions(joint_angles['left'])
            right.set_joint_positions(joint_angles['right'])
            print "updated positions"
    print "Rospy shutdown, exiting loop."
    return True


def main():
    """
    Note: This version of simply drives the joints towards the next position at
    each time stamp. Because it uses Position Control it will not attempt to
    adjust the movement speed to hit set points "on time".
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-r', '--rate', type=int, default=10,
        help='rate to sample the joint positions'
    )

    parser.add_argument(
        '-t', '--test', type=bool, default=False,
        help='use hardcoded test joint angles'
    )
    parser.add_argument(
        '-u', '--user', type=int, default=1,
        help='kinect user number to use'
    )

    parser.add_argument(
        '-m', '--mirrored', type=bool, default=False,
        help='mirror the teleoperators movements'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("teleoperation")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    teleoperate(args.rate, args.user, args.test, args.mirrored)

if __name__ == '__main__':
    main()
