#!/usr/bin/env python3
# Copyright by BeeX [2026]
"""Manual arm commands. Each one takes the arm from whatever the pick is doing.

    rosrun final_v1 arm.py where                      print every joint, degrees as reported, and the jaw in mm
    rosrun final_v1 arm.py point 0.20 0.05 0.25       put the grasp point there: metres in arm_base
    rosrun final_v1 arm.py joints 300 100 30 0        base shoulder elbow wrist, degrees as reported
    rosrun final_v1 arm.py joint elbow 45             one joint, the others stay
    rosrun final_v1 arm.py jaw open | close | 5.0     or an opening in mm
    rosrun final_v1 arm.py home                       the driver's own ramp home
    rosrun final_v1 arm.py release                    standby: the arm goes limp

point, joints and joint go through the pick: planned and checked against the joint limits, the hull and the last look's
obstacle map, then followed as a pick's own motion is. Ctrl-C while one runs stops the arm. The others go straight to the
driver."""
import math
import sys

import rospy
from final_v1.msg import PickState
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

JOINTS = ['base', 'shoulder', 'elbow', 'wrist']


def call(name):
    try:
        rospy.wait_for_service(name, timeout=2.0)
        reply = rospy.ServiceProxy(name, Trigger)()
        print('%s: %s' % (name, reply.message or ('ok' if reply.success else 'refused')))
        return reply.success
    except (rospy.ROSException, rospy.ServiceException):
        print('%s: no answer' % name)
        return False


def reading(names):
    msg = rospy.wait_for_message('joint_states', JointState, timeout=2.0)
    age = (rospy.Time.now() - msg.header.stamp).to_sec()
    if age > 1.0:
        print('WARNING: this reading is %.0f s old: the arm is not answering the driver' % age)
    return [msg.position[msg.name.index(n)] for n in names]


def go(topic, msg):
    """Hands a goal to the pick and waits for it to say how the move ended."""
    states = []
    rospy.Subscriber('pick/state', PickState, states.append)
    pub = rospy.Publisher(topic, type(msg), queue_size=1)
    for _ in range(50):
        if pub.get_num_connections():
            break
        rospy.sleep(0.1)
    moving = bool(states) and states[-1].state == 'MANUAL'  # a move already under way is replanned without a new state
    del states[:]
    pub.publish(msg)
    taken = rospy.Time.now()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        if not moving and not any(s.state == 'MANUAL' for s in states):
            if rospy.Time.now() - taken > rospy.Duration(5.0):
                print('the pick did not take the command: is pick.launch up, and the arm allowed to move?')
                return False
        elif states and states[-1].state != 'MANUAL':
            print('%s: %s' % (states[-1].state, states[-1].message))
            return states[-1].state == 'READY'
    return False


def main():
    args = sys.argv[1:]
    if not args or args[0] not in ('where', 'point', 'joints', 'joint', 'jaw', 'home', 'release'):
        print(__doc__)
        return 1
    rospy.init_node('arm', anonymous=True, disable_signals=True)
    if not rospy.has_param('/robot'):
        print('no /robot parameters: is pick.launch running?')
        return 1
    robot = rospy.get_param('/robot')
    arm, jaws = robot['arm'], robot['jaws']
    names, jaw = [arm['joints'][j] for j in JOINTS], arm['joints']['jaw']
    if args[0] == 'where':
        now = reading(names + [jaw])
        print(' '.join('%s %.1f' % (j, math.degrees(r)) for j, r in zip(JOINTS, now)) + ' jaw %.1f mm' % (now[-1] * 1000.0))
        return 0

    if args[0] in ('home', 'jaw') and not rospy.get_param('/pick/arm/move', True):
        print('refused: the arm is not to move (pick:=false)')
        return 1
    if args[0] in ('home', 'release', 'jaw') and call('pick/stop'):
        rospy.sleep(0.3)  # the pick releases the arm on its next tick; a command sent before that would be cancelled by it
    if args[0] == 'home':
        return 0 if call('driver/home') else 1
    if args[0] == 'release':
        return 0 if call('driver/standby') else 1
    if args[0] == 'jaw':
        if args[1] in ('open', 'close'):
            return 0 if call('driver/' + args[1]) else 1
        opening = float(args[1]) / 1000.0
        if not jaws['limits'][0] <= opening <= jaws['limits'][1]:
            print('jaw %.1f mm is outside %.1f to %.1f mm' % (opening * 1000.0, jaws['limits'][0] * 1000.0, jaws['limits'][1] * 1000.0))
            return 1
        rospy.Publisher('driver/targets', JointState, queue_size=1, latch=True).publish(JointState(name=[jaw], position=[opening]))
        rospy.sleep(1.0)
        return 0

    try:
        if args[0] == 'point':
            return 0 if go('pick/point', Point(*[float(a) for a in args[1:4]])) else 1
        goals = [math.degrees(r) for r in reading(names)]
        if args[0] == 'joints':
            goals = [float(a) for a in args[1:]]
            assert len(goals) == len(JOINTS)
        else:
            goals[JOINTS.index(args[1])] = float(args[2])
    except (AssertionError, IndexError, TypeError, ValueError):
        print(__doc__)
        return 1
    return 0 if go('pick/posture', JointState(name=names, position=[math.radians(g) for g in goals])) else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('interrupted: stopping the arm')
        call('pick/stop')
        sys.exit(1)
    except rospy.ROSException:
        print('no joint readings: is the driver running?')
        sys.exit(1)
