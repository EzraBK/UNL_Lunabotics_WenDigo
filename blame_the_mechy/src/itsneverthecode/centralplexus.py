#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from roboclaw_driver.msg import SpeedCommand
global robocommand

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs to commands for the roboclaw
# axis 1 aka left stick vertical controls forwards/ backwards
m1_speed = 0
m2_speed = 0
accel_var = 4000
max_secs_var = 1

'''
 .------------------------.
(  MINE ME, LUNABOTICS!!!! )    ..
 `------------------------'   .' /
                        O    /  ;
                          o i  OO
                           C    `-.
                           |    <-'
                           (  ,--.
                            V  \_)
                             \  :
                              `._\.
'''

def callback(data):
    global m1_speed
    global m2_speed
    global max_secs_var
    global accel_var
    #data.axes[2]
    #print data.buttons[0]
    if data.buttons[0] == 1:
        m2_speed = 0
        m1_speed = 0
        print "STOP"
    else:
        m2_speed = 1000*data.axes[1]
        m1_speed = 1000*data.axes[1]
        print "GO"
    if data.buttons[1] == 1:
        print "NATE IS GREAT"
        #print "assigning new values!"    
    

# Intializes everything
def start():
    # publishing to "roboclaw/speed_command" 
    global m1_speed
    global m2_speed
    global max_secs_var
    global accel_var
    rospy.init_node('Joy2Turtle')
    robocommand = SpeedCommand()
    FourWD = rospy.get_param("motor_control/4WD")
    #pub = rospy.Publisher('roboclaw/speed_command', SpeedCommand,queue_size=10)
    motor_right_pub = rospy.Publisher('roboclaw/speed_command/right', SpeedCommand, queue_size = 1)
    motor_left_pub = rospy.Publisher('roboclaw/speed_command/left', SpeedCommand, queue_size = 1)
    

    
    '''
    create the other publishers for 
    '''
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber('joy', Joy, callback)
    # starts the node
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msgR = SpeedCommand()
        msgL = SpeedCommand()
        msgR.m1_qpps = -m1_speed
        msgR.m2_qpps = m1_speed
        msgR.max_secs = max_secs_var
        msgL.m1_qpps = m2_speed
        msgL.m2_qpps = m2_speed
        msgL.max_secs = max_secs_var
        motor_right_pub.publish(msgR)
        motor_left_pub.publish(msgL)
        
        #robocommand.m1_qpps = m1_speed
        #robocommand.m2_qpps = m2_speed
        #robocommand.max_secs = max_secs_var
        #robocommand.accel = accel_var
        #pub.publish(robocommand)
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    start()


