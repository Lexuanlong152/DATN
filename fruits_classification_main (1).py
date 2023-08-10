#!/usr/bin/env python3
import os
import sys
import cv2
import math
import rospy
import numpy as np
import threading
import queue
import hiwonder
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from jetmax_control.msg import SetServo
from yolov5_tensorrt import Yolov5TensorRT

import Jetson.GPIO as GPIO
import justkinematics
import serial
import time


ROS_NODE_NAME = "fruits_classification"
"""
TRT_ENGINE_PATH = os.path.join(sys.path[0], "waste_v5_160.trt")
TRT_INPUT_SIZE = 160
TRT_CLASS_NAMES = ('Banana Peel', 'Broken Bones', 'Cigarette End', 'Disposable Chopsticks',
                   'Ketchup', 'Marker', 'Oral Liquid Bottle', 'Plate',
                   'Plastic Bottle', 'Storage Battery', 'Toothbrush', 'Umbrella')
TRT_NUM_CLASSES = 12
WASTE_CLASSES = {
    'food_waste': ('Banana Peel', 'Broken Bones', 'Ketchup'),
    'hazardous_waste': ('Marker', 'Oral Liquid Bottle', 'Storage Battery'),
    'recyclable_waste': ('Plastic Bottle', 'Toothbrush', 'Umbrella'),
    'residual_waste': ('Plate', 'Cigarette End', 'Disposable Chopsticks'),
}
COLORS = {
    'recyclable_waste': (0, 0, 255),
    'hazardous_waste': (255, 0, 0),
    'food_waste': (0, 255, 0),
    'residual_waste': (80, 80, 80)
}
TARGET_POSITIONS = {
    'recyclable_waste': (163, -60, 65, 65),
    'hazardous_waste': (163, -10, 65, 85),
    'food_waste': (163, -10 + 52, 65, 100),
    'residual_waste': (163, -10 + 52 * 2, 65, 118)
}
"""

"""
TRT_ENGINE_PATH = os.path.join(sys.path[0], "train_data_fruits_demo.trt")
TRT_INPUT_SIZE = 160
TRT_CLASS_NAMES = ('Chom Chom')
TRT_NUM_CLASSES = 1
WASTE_CLASSES = {
    'chomchom': ('Chom Chom')
}
COLORS = {
    'chomchom': (0, 255, 255)
}
TARGET_POSITIONS = {
    'chomchom': (163, -60, 65, 65)
}
"""

#TRT_ENGINE_PATH = os.path.join(sys.path[0], "last_training_fruits.trt")

TRT_ENGINE_PATH = os.path.join(sys.path[0], "last_training_fruits.trt")
TRT_INPUT_SIZE = 160
#TRT_CLASS_NAMES = ('Banana', 'Lemon')
TRT_CLASS_NAMES = ('Banana', 'Lemon','mangosteen','apple','greenMS')
TRT_NUM_CLASSES = 5
WASTE_CLASSES = {
    'banana': ('Banana'),
    'lemon': ('Lemon'),
    'mangosteen': ('mangosteen'),
    'apple': ('apple'),
    'greenms': ('greenMS')
}
COLORS = {
    'banana': (0, 255, 0),
    'lemon': (255, 0, 0),
    'mangosteen': (255, 0, 0),
    'apple': (255, 0, 0),
    'greenms' : (255, 0, 0)
}
TARGET_POSITIONS = {
    'banana': 1,
    'lemon': 1,
    'mangosteen': -1,
    'apple': -1,
    'greenms':1
}
GRIPPER_ANGLES = {
    'banana': 80,
    'lemon': 75,
    'mangosteen': 80,
    'apple': 70,
    'greenms' : 80
}

def camera_to_world(cam_mtx, r, t, img_points):
    inv_k = np.asmatrix(cam_mtx).I
    r_mat = np.zeros((3, 3), dtype=np.float64)
    cv2.Rodrigues(r, r_mat)
    # invR * T
    inv_r = np.asmatrix(r_mat).I  # 3*3
    transPlaneToCam = np.dot(inv_r, np.asmatrix(t))  # 3*3 dot 3*1 = 3*1
    world_pt = []
    coords = np.zeros((3, 1), dtype=np.float64)
    for img_pt in img_points:
        coords[0][0] = img_pt[0][0]
        coords[1][0] = img_pt[0][1]
        coords[2][0] = 1.0
        worldPtCam = np.dot(inv_k, coords)  # 3*3 dot 3*1 = 3*1
        # [x,y,1] * invR
        worldPtPlane = np.dot(inv_r, worldPtCam)  # 3*3 dot 3*1 = 3*1
        # zc
        scale = transPlaneToCam[2][0] / worldPtPlane[2][0]
        # zc * [x,y,1] * invR
        scale_worldPtPlane = np.multiply(scale, worldPtPlane)
        # [X,Y,Z]=zc*[x,y,1]*invR - invR*T
        worldPtPlaneReproject = np.asmatrix(scale_worldPtPlane) - np.asmatrix(transPlaneToCam)  # 3*1 dot 1*3 = 3*3
        pt = np.zeros((3, 1), dtype=np.float64)
        pt[0][0] = worldPtPlaneReproject[0][0]
        pt[1][0] = worldPtPlaneReproject[1][0]
        pt[2][0] = 0
        world_pt.append(pt.T.tolist())
    return world_pt


class FruitClassification:
    def __init__(self):
        self.lock = threading.RLock()
        self.is_running = False
        self.moving_box = None
        self.image_sub = None
        self.heartbeat_timer = None
        self.runner = None
        self.count = 0
        self.camera_params = None
        self.K = None
        self.R = None
        self.T = None
        self.is_target = False

    def reset(self):
        self.is_running = False
        self.moving_box = None
        self.image_sub = None
        self.heartbeat_timer = None
        self.runner = None
        self.count = 0

    def load_camera_params(self):
        self.camera_params = rospy.get_param('/camera_cal/card_params', self.camera_params)
        if self.camera_params is not None:
            self.K = np.array(self.camera_params['K'], dtype=np.float64).reshape(3, 3)
            self.R = np.array(self.camera_params['R'], dtype=np.float64).reshape(3, 1)
            self.T = np.array(self.camera_params['T'], dtype=np.float64).reshape(3, 1)

def moving():
    global flag_target_ok
    global motor_state
    global flag_seek_left
    global flag_seek_right

    try:
        c_x, c_y, waste_class_name = state.moving_box
        cur_x, cur_y, cur_z = jetmax.position

        x, y, _ = camera_to_world(state.K, state.R, state.T, np.array([c_x, c_y]).reshape((1, 1, 2)))[0][0]
        print("dist", x, y)
        rospy.loginfo("dist : " + str( x) + " ; " + str ( y) )

        t = math.sqrt(x * x + y * y + 140 * 140) / 140

        new_x, new_y = cur_x + x, cur_y + y
        nn_new_x = new_x + 15
        arm_angle = math.atan(new_y / new_x) * 180 / math.pi
        if arm_angle > 0:
            arm_angle = (90 - arm_angle)
        elif arm_angle < 0:
            arm_angle = (-90 - arm_angle)
        else:
            pass

        #-----Hung------

        # Move the gripper to item's position and open the gripper
        #hiwonder.pwm_servo2.set_position(45,0.5)
        #move_to_pos((x, y, -100),t)
        print("item :", waste_class_name )
        print("t1 : ",t)
        move_and_open((x, y, -110),t*0.5)
        # Wait 0.1 second
        rospy.sleep(0.1)
        # close gripper

        close_gripper(GRIPPER_ANGLES[waste_class_name],1)

        # Wait 0.5 second
        rospy.sleep(0.1)

        # Neu vat the bi rot / gap chua duoc vat the
        state_gripper = GPIO.input(9)
        if state_gripper == 0:
           print("state_gripper: pass")
           #move to target position
           target_pos = TARGET_POSITIONS[waste_class_name]
           cur_x, cur_y, cur_z = jetmax.position
           t = math.sqrt((cur_x - jetmax.ORIGIN[0]) ** 2 + (cur_y - jetmax.ORIGIN[1]) ** 2+ 140 * 140) / 140
           print("t2 : ",t)
           move_to_target(t*0.5,target_pos)
           # Wait 0.5 second
           rospy.sleep(0.1)

        
           if motor_state == STOP:
              if TARGET_POSITIONS[waste_class_name] < 0:
                  motor_cmd("LEFT")
                  flag_target_ok == False
                  motor_state = TARGET_LEFT
                  flag_seek_left = None
                  flag_seek_right = None
                  print("TARGET_LEFT")

              if TARGET_POSITIONS[waste_class_name] > 0:
                  motor_cmd("RIGHT")
                  flag_target_ok == False
                  motor_state = TARGET_RIGHT
                  flag_seek_left = None
                  flag_seek_right = None
                  print("TARGET_RIGHT")
        else: 
            motor_state == STOP
            open_gripper(0.25)
            rospy.sleep(0.2)
            move_to_pos((0,0,0),1) # Hung
            rospy.sleep(1+0.1)
            print("GRIPPER: FAIL")

    finally:
        state.moving_box = None
        state.runner = None
        print("TARGET")


def image_proc():
    global state_scroll
    global flag_ok
    global flag_mid
    global motor_dir
    global motor_state
    global flag_seek_left
    global flag_seek_right
    global flag_target_ok

    ros_image = image_queue.get(block=True)

    if (motor_state == TARGET_LEFT) or (motor_state == TARGET_RIGHT) : 
        if flag_target_ok == True:
           flag_target_ok = False
           # release item
           open_gripper(0.25)        
           # Wait 0.5 second
           rospy.sleep(0.2)
           _t = 0.5
           move_to_pos((0,0,0),_t) # Hung
           rospy.sleep(_t+0.1)
           state.moving_box = None
           state.runner = None
           print("FINISHED")
           flag_seek_left = None
           flag_seek_right = None
           if (motor_state == TARGET_LEFT):
               motor_state = SEEK_RIGHT
               print("SEEK_RIGHT")
           elif (motor_state == TARGET_RIGHT):
               motor_state = SEEK_LEFT
               print("SEEK_LEFT")
 
           return
    


    if state.is_running is False or state.runner is not None:
        image_pub.publish(ros_image)
        #image_pub2.publish(ros_image)
        return


    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
    outputs = yolov5.detect(image)
    boxes, confs, classes = yolov5.post_process(image, outputs, 0.65)
    width = image.shape[1]
    height = image.shape[0]
    cards = []
    for box, cls_conf, cls_id in zip(boxes, confs, classes):
        x1 = int(box[0] / TRT_INPUT_SIZE * width)
        y1 = int(box[1] / TRT_INPUT_SIZE * height)
        x2 = int(box[2] / TRT_INPUT_SIZE * width)
        y2 = int(box[3] / TRT_INPUT_SIZE * height)
        waste_name = TRT_CLASS_NAMES[cls_id]
        waste_class_name = ''
        for k, v in WASTE_CLASSES.items():
            if waste_name in v:
                waste_class_name = k
                break
        
        if y1 > 330 and x1 >540:
            continue

        cards.append((cls_conf, x1, y1, x2, y2, waste_class_name))
        image = cv2.putText(image, waste_name + " " + str(float(cls_conf))[:4], (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLORS[waste_class_name], 2)
        image = cv2.rectangle(image, (x1, y1), (x2, y2), COLORS[waste_class_name], 3)
    

    if len(cards) == 0 :
        state.count = 0
        state.moving_box = None

        # Control bang chuyen
        if (motor_state == HOME):
            motor_state = SEEK_LEFT
            print("HOME")

        elif (motor_state == SEEK_LEFT):
            motor_cmd("SL")
            flag_ok = False
            flag_seek_left = False
            flag_seek_right = None
            motor_state = WAIT_ITEM
            print("WAIT_ITEM")

        elif (motor_state == WAIT_ITEM):
            if flag_ok == True:
                flag_ok = False

                if flag_seek_left == False and flag_seek_right == False:
                   motor_state = MID
                   print("MIDDLE")
                elif flag_seek_left == False:
                   motor_state = SEEK_RIGHT
                   print("SEEK_RIGHT")


        elif (motor_state == SEEK_RIGHT):
            motor_cmd("SR")
            flag_seek_right = False
            motor_state = WAIT_ITEM

            if flag_seek_left == None:
               motor_state = SEEK_LEFT
               print("SEEK_LEFT")

        elif (motor_state == STOP):
            flag_seek_left = None
            flag_seek_right = None

        elif (motor_state == MID):
            motor_cmd("MID")
            flag_ok = False
            motor_state = STOP
        
    else:
        if state.moving_box is None:
            moving_box = max(cards, key=lambda card: card[0])
            conf, x1, y1, x2, y2, waste_class_name = moving_box
            c_x, c_y = (x1 + x2) / 2, (y1 + y2) / 2
            state.moving_box = c_x, c_y, waste_class_name
        else:
            l_c_x, l_c_y, l_waste_class_name = state.moving_box
            moving_box = min(cards, key=lambda card: math.sqrt((l_c_x - card[1]) ** 2 + (l_c_y - card[2]) ** 2))

            conf, x1, y1, x2, y2, waste_class_name = moving_box
            c_x, c_y = (x1 + x2) / 2, (y1 + y2) / 2

            image = cv2.rectangle(image, (x1, y1), (x2, y2), (255, 255, 255), 6)
            image = cv2.circle(image, (int(c_x), int(c_y)), 1, (255, 255, 255), 10)

            if math.sqrt((l_c_x - c_x) ** 2 + (l_c_y - c_y) ** 2) > 30:
                state.count = 0
            else:
                c_x = l_c_x * 0.2 + c_x * 0.8
                c_y = l_c_y * 0.2 + c_y * 0.8
                state.count += 1
            state.moving_box = c_x, c_y, waste_class_name

            if state.count > 15 and (motor_state == WAIT_ITEM):
               # TURN OFF "Bang chuyen"
               motor_cmd("0")
               motor_state = STOP

            if state.count > 30:
                state.count = 0
                GPIO.output(4,GPIO.LOW)
                rospy.sleep(0.2)
                GPIO.output(4,GPIO.HIGH)
                state.runner = threading.Thread(target=moving, daemon=True)
                state.runner.start()

    rgb_image = image.tostring()
    ros_image.data = rgb_image
    image_pub.publish(ros_image)


def image_callback(ros_image):
    try:
        image_queue.put_nowait(ros_image)
    except queue.Full:
        pass


def enter_func(msg):
    rospy.loginfo("enter")
    exit_func(msg)
    jetmax.go_home()
    state.reset()
    state.load_camera_params()
    state.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, image_callback)
    return TriggerResponse(success=True)


def exit_func(msg):
    global motor_state
    global flag_ok
 
    rospy.loginfo("exit")
    motor_state = HOME
    flag_ok = False
    state.is_running = False
    try:
        state.heartbeat_timer.cancel()
    except:
        pass
    if isinstance(state.runner, threading.Thread):
        state.runner.join()
    if isinstance(state.image_sub, rospy.Subscriber):
        rospy.loginfo('unregister image')
        state.image_sub.unregister()
    rospy.ServiceProxy('/jetmax/go_home', Empty)()
    rospy.Publisher('/jetmax/end_effector/sucker/command', Bool, queue_size=1).publish(data=False)
    rospy.Publisher('/jetmax/end_effector/servo1/command', SetServo, queue_size=1).publish(data=90, duration=0.5)
    
    
    return TriggerResponse(success=True)


def set_running(msg: SetBoolRequest):
    global motor_state

    motor_state == HOME
    if msg.data:
        rospy.loginfo("start running")
        state.is_running = True
    else:
        rospy.loginfo("stop running")
        state.is_running = False
    return SetBoolResponse(success=True)


def heartbeat_timeout_cb():
    rospy.loginfo('heartbeat timeout. exiting...')
    rospy.ServiceProxy('/%s/exit' % ROS_NODE_NAME, Trigger)


def heartbeat_srv_cb(msg: SetBoolRequest):
    rospy.logdebug("Heartbeat")
    try:
        state.heartbeat_timer.cancel()
    except:
        pass
    if msg.data:
        state.heartbeat_timer = threading.Timer(5, heartbeat_timeout_cb)
        state.heartbeat_timer.start()
    return SetBoolResponse(success=msg.data)

#------------------------------------------------#
#------------ CODE by Hung-----------------------#
#------------------------------------------------#
GPIO.setmode(GPIO.BCM)
GPIO.setup(9, GPIO.IN)
GPIO.setup(4, GPIO.OUT, initial=GPIO.HIGH)

L0 = 84.4
L1 = 8.14
L2 = 128.4
L3 = 138.0
L4 = 16.8

state_relay = 1
state_complete = 1
time_read = 0
left_right = 0

LEFT  = 1
RIGHT = -1

control_motor = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)


state_scroll = 0

HOME = 0
SEEK_LEFT = 1
SEEK_RIGHT = 2
WAIT_ITEM = 3
STOP = 4
TARGET_LEFT = 5
TARGET_RIGHT = 6
WAIT_TARGET = 7
MID = 8

flag_ok = False
motor_dir = 1
motor_state = HOME
flag_mid = False
flag_seek_left = None
flag_seek_right = None
flag_target_ok = False
motor_position = 212
time_out = time.time()

def return_valid_x(angle_raw):
    alpha=6
    if angle_raw < 31:
       return (30- 30+alpha)
    if angle_raw > 209:
       return (210- 30+alpha)
    return (angle_raw - 30+alpha)

def set_pos_x(angle, duration):
    hiwonder.pwm_servo1.set_position(return_valid_x(angle), duration)

def go_home(duration):
    alpha = 6
    angles = justkinematics.inverse(L4, (0,-(L1 + L3 + L4), L0 + L2))
    jetmax.go_home(duration)
    set_pos_x(angles[0],duration)
    hiwonder.pwm_servo2.set_position(90,1)
    rospy.sleep(duration+0.1)

def set_pos(postion, duration):
    angles = justkinematics.inverse(L4, postion)
    jetmax.set_position(postion,duration)
    set_pos_x(angles[0],duration*0.8)
    rospy.sleep(duration+0.1)

def open_gripper(duration):
    hiwonder.pwm_servo2.set_position(45,duration)
    rospy.sleep(duration+0.1)

def close_gripper(angle,duration):
    #global state_relay
    if angle < 45:
       return
    del_angle = angle -45
    _time = 0.03
    for i in range (int(del_angle/2)):
        hiwonder.pwm_servo2.set_position(45+i*2,_time)
        
        rospy.sleep(_time+0.01)
        state_relay = GPIO.input(9)        
        rospy.sleep(0.01)
        if (state_relay == 0 ):
             print("current_state : ", str(state_relay))
             break;
        
    rospy.sleep(0.5)

def scroll_ON():
    global state_scroll
    global motor_dir

    if ( state_scroll == 0):
        state_scroll = 1
        print("Bang chuyen: ON ")
        # send message by rs485
        if motor_dir > 0:
           _event = threading.Thread(target=motor_cmd,args=("RUN",), daemon=True)    
           _event.start()
           rospy.sleep(0.2)
           motor_dir = -1
           _event = threading.Thread(target=motor_cmd,args=("65500",), daemon=True)    
           _event.start()
           rospy.sleep(0.2)

        # send message by rs485
        if motor_dir < 0:
           _event = threading.Thread(target=motor_cmd,args=("RUN",), daemon=True)    
           _event.start()
           rospy.sleep(0.2)
           motor_dir = 1
           _event = threading.Thread(target=motor_cmd,args=("500",), daemon=True)    
           _event.start()
           rospy.sleep(0.2)

def scroll_OFF():
    global state_scroll
    global motor_dir

    if ( state_scroll == 1):
       state_scroll = 0
       print("Bang chuyen: OFF ")
    # send message by rs485
       _event = threading.Thread(target=motor_cmd,args=("0",), daemon=True)    
       _event.start()
       rospy.sleep(0.2)

def scroll_state():
    global state_scroll
    return state_scroll

def move_and_open(position,_duration):
    open_gripper(0.5)
    move_to_pos(position,_duration)    
    rospy.sleep(0.1)

def set_pos_parallel(postion, duration):
    angles = justkinematics.inverse(L4, postion)
    jetmax.set_position(postion,duration)
    set_pos_x(angles[0],duration*0.6)

def move_to_target(duration,direct):
    global motor_dir
    global motor_state
    global flag_seek_left
    global flag_seek_right

    home_x = 0
    home_y = -(L1 + L3 + L4)
    home_z = L0 + L2
    delta_z = 0
    delta_x = home_y*0.7
    delta_y = home_y*0.7
    set_pos_parallel((delta_x*direct,delta_y,home_z+delta_z),duration)
    rospy.sleep(duration)

    state_relay = GPIO.input(9)
    if state_relay == 1:
       return move_and_open((0,0,0),0.5)

    delta_x = home_y
    delta_y = home_x
    delta_z = -50
    set_pos_parallel((delta_x*direct,delta_y,home_z),duration)
    rospy.sleep(duration*0.9)

    state_relay = GPIO.input(9)
    if state_relay == 1:
       return move_and_open((0,0,0),0.5)

    set_pos_parallel(((delta_x-20)*direct,delta_y,home_z+delta_z),duration)
    motor_dir = direct 
    rospy.sleep(duration)

def move_to_pos(position, duration):
    home_x = 0
    home_y = -(L1 + L3 + L4)
    home_z = L0 + L2
    delta_z = position[2]+20
    delta_y = position[1]
    delta_x = position[0]
    _duration = 2    
    set_pos((home_x+delta_x,home_y+delta_y,home_z+delta_z),_duration)

def motor_write(cmd):
    control_motor.write(bytes(str(cmd), encoding='utf-8'))
    rospy.sleep(0.1)

def motor_cmd(cmd):
    _event = threading.Thread(target=motor_write,args=(str(cmd),), daemon=True)    
    _event.start()
    rospy.sleep(0.2)


def motor_wait():
    global flag_ok
    while flag_ok == False:
        data = ""
        while control_motor.inWaiting() > 0:
           data = control_motor.readline().decode(encoding='utf-8')
           if str("IDLE") in data :
              flag_ok = True
              print("IDLE")

#------------------------------------------------#
#------------------------------------------------#
#------------------------------------------------#

if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)
    image_queue = queue.Queue(maxsize=2)
    state = FruitClassification()
    state.load_camera_params()
    if state.camera_params is None:
        rospy.logerr("Can not load camera parameters")
        sys.exit(-1)
    jetmax = hiwonder.JetMax()
    #sucker = hiwonder.Sucker()
    go_home(1)
    yolov5 = Yolov5TensorRT(TRT_ENGINE_PATH, TRT_INPUT_SIZE, TRT_NUM_CLASSES)
    #ROS_NODE_NAME2 = "fruit_classification"
    #image_pub2 = rospy.Publisher('/%s/image_result' % ROS_NODE_NAME2, Image, queue_size=1)  # register result image pub
    image_pub = rospy.Publisher('/%s/image_result' % ROS_NODE_NAME, Image, queue_size=1)  # register result image pub
    enter_srv = rospy.Service('/%s/enter' % ROS_NODE_NAME, Trigger, enter_func)
    exit_srv = rospy.Service('/%s/exit' % ROS_NODE_NAME, Trigger, exit_func)
    running_srv = rospy.Service('/%s/set_running' % ROS_NODE_NAME, SetBool, set_running)
    heartbeat_srv = rospy.Service('/%s/heartbeat' % ROS_NODE_NAME, SetBool, heartbeat_srv_cb)
    #move_to_pos((0,0,-105),3)
    open_gripper(0.5)
    
    #GPIO.output(4,GPIO.LOW) # turn on LED    
    motor_wait()
    flag_ok = False
    motor_cmd("HOME")
    prev_state = HOME

    while True:
        try:            

          
            #state_relay = GPIO.input(9)

            if not (prev_state == motor_state ) or (motor_state == HOME) or (prev_state == MID) :
                  time_out = time.time()
                  if not ( prev_state == MID and motor_state == STOP ):
                     prev_state = motor_state

            if ( time.time() - time_out > 15)  :
                  #flag_ok = False
                  #motor_dir = 1
                  #motor_state = HOME
                  #flag_mid = False
                  #flag_seek_left = None
                  #flag_seek_right = None
                  #flag_target_ok = False
                  #motor_position = 212
                  time_out = time.time()
                  #open_gripper(0.5)
                  #motor_cmd("HOME")
                  #move_to_pos((0,0,0),1) # Hung
                  #rospy.sleep(5)
                  print("TIME OUT")              
            
            while control_motor.inWaiting() > 0:
                 data = control_motor.readline().decode(encoding='utf-8')                 

                 if str("STOP:") in data :
                    motor_position = int((str(data).split(":"))[1])
                    print("Pos: ",motor_position)

                 if str("IDLE") in data :
                    time_out = time.time()
                    flag_ok = True
                    print("IDLE")

                 if str("TL") in data or ( str("TR") in data ):
                    flag_target_ok = True
                    print("flag_target_ok")

            image_proc()

            if rospy.is_shutdown():
                break

        except Exception as e:
            rospy.logerr(e)
            break
    #GPIO.output(4,GPIO.HIGH) # turn off LED











