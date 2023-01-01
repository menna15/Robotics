#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, os

import tty, termios

MAX_LIN_VEL = 3.85
MAX_ANG_VEL = 2.85


LIN_VEL_STEP_SIZE = 0.03   
ANG_VEL_STEP_SIZE = 0.01

msg = """
Control Your Robot
------------------
Moving around:
        w
   a  space  d
        s
w/s : increase/decrease linear velocity 
a/d : increase/decrease angular velocity
space key : force stop
CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)



def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input



if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('control')
    pub = rospy.Publisher('/robot/robotnik_base_control/cmd_vel', Twist, queue_size = 10)

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0

    try:
        print(msg)
        while True:
            key = getKey()
            if key == 'w' :
                target_linear_vel = constrain(target_linear_vel+LIN_VEL_STEP_SIZE,-MAX_LIN_VEL,MAX_LIN_VEL)
                status = 0
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 's' :
                target_linear_vel = constrain(target_linear_vel-LIN_VEL_STEP_SIZE,-MAX_LIN_VEL,MAX_LIN_VEL)
                status = 0
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'a' :
                target_angular_vel = constrain(target_angular_vel + ANG_VEL_STEP_SIZE,-MAX_ANG_VEL,MAX_ANG_VEL)
                status = 0
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'd' :
                target_angular_vel = constrain(target_angular_vel - ANG_VEL_STEP_SIZE,-MAX_ANG_VEL,MAX_ANG_VEL)
                status = 0
                print(vels(target_linear_vel,target_angular_vel))
            elif key == ' ':
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))

            elif (key == '\x03'):
                    break
            elif(key == ''):
                if target_linear_vel != 0.0:
                    status +=1
                    print(status)
                    target_linear_vel = constrain(target_linear_vel-LIN_VEL_STEP_SIZE,-MAX_LIN_VEL,MAX_LIN_VEL)
                    if status == 10:
                        target_linear_vel = 0.0
                        target_angular_vel = 0.0
                    print(vels(target_linear_vel, target_angular_vel))
                elif target_angular_vel != 0.0:
                    status +=1
                    print(status)
                    target_angular_vel = constrain(target_angular_vel-ANG_VEL_STEP_SIZE,-MAX_ANG_VEL,MAX_ANG_VEL)
                    if status == 10:
                        target_linear_vel = 0.0
                        target_angular_vel = 0.0
                    print(vels(target_linear_vel, target_angular_vel))


            twist = Twist()

            twist.linear.x = target_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = target_angular_vel

            pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)