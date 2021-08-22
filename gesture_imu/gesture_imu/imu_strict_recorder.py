# /freertos_string_publisher 

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import pickle
import cv2
import numpy as np
import random
import time
import os 
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
import json

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_recorder')
        self.get_logger().info('Creating Node...') 
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/freertos_string_publisher',
            self.listener_callback,
            20)
        self.subscription
        parameter_descriptor = ParameterDescriptor(description='Sets the label of the recorder. 0: None, 1: Forward, 2: Backward, 3: Left, 4" Right, 5:,Clockwise, 6: C-Clockwise')
        self.declare_parameter('label', 1, parameter_descriptor)

        self.get_logger().info('Creating Recorder...') 
        self.mov_rec = movementRecorder()
        self.recorded_list = []
        self.recorded_label = []
        self.i=0
        self.timet = time.time()

        self.label = self.get_parameter('label').value
        self.end_count = 0

        self.last_save =np.array([0,0])
        self.check_data = []

        self.get_logger().info('Starting...') 

    def listener_callback(self,msg):
        if self.end_count >= 3:
            try:
                files = os.listdir('data/simple')
                files = [int(x.split('.')[0][2]) for x in files]
                listname = 'data/simple/'+str(self.label)+'_'+str(max(files)+1)
            except:
                listname = 'data/simple/'+str(self.label)+'_1'
            with open(listname+'.pkl', 'wb') as f:
                pickle.dump([self.recorded_list ,self.recorded_label],f)
            self.get_logger().info('Saved list...%s' % (listname))
            cv2.destroyWindow('Frame')
            self.destroy_node()
            exit()

        cur_save = np.array([msg.data[0], msg.data[3]])
        self.check_data = cur_save != self.last_save
        self.last_save = cur_save


        self.mov_rec.vid(self.i,self.label,self.check_data)

        if msg.data[6]==1:
            self.mov_rec.add_val(msg.data)
        elif self.mov_rec.lower_bound():
            self.mov_rec.mov_stack = []
            self.end_count += 1
        elif self.mov_rec.check_out():
            self.recorded_list.append(self.mov_rec.record_vals())
            self.recorded_label.append(self.label)
            self.i+=1
            self.end_count = 0 
        #else:
         #   return

        #self.get_logger().info('I heard stuff...') # %i' % (self.i))
        #fps = 60/(time.time()-self.timet)
        #self.timet = time.time()
        #self.get_logger().info('FPS:  %i' % (fps))


class movementRecorder:
    def __init__(self):
        self.movements = ['None','Forward','Backward','Left','Right','Clockwise','C-Clockwise']
        self.mov_stack = []
        self.font = cv2.FONT_HERSHEY_SIMPLEX

    def add_val(self,data):
        self.mov_stack.append(data[:6])

    def lower_bound(self):
        return len(self.mov_stack)<10 and len(self.mov_stack)>0

    def check_out(self):
        return len(self.mov_stack)>0

    def record_vals(self):
        temp_hold = self.mov_stack
        self.mov_stack = []
        return temp_hold

    def vid(self,rec_len,label,check_data):

        img = np.zeros((512, 512, 3), dtype = "uint8")
        img[:,:,0]=114
        img[:,:,1]=95
        img[:,:,2]=89

        textsize = cv2.getTextSize(self.movements[label], self.font, 3, 6)[0]
        cv2.putText(img,
            self.movements[label], 
            (int((img.shape[1] - textsize[0])/2),150),
            self.font, 
            3,
            (80,211,195),
            6)

        textsize = cv2.getTextSize(str(rec_len), self.font, 3, 6)[0]
        cv2.putText(img,
            str(rec_len), 
            (int((img.shape[1] - textsize[0])/2),275),
            self.font, 
            3,
            (80,211,195),
            6)
        text = "Accel: "+str(check_data[0])+"    Gyro: "+str(check_data[1])
        textsize = cv2.getTextSize(text, self.font, 1, 2)[0]
        cv2.putText(img,
            text, 
            (int((img.shape[1] - textsize[0])/2),475),
            self.font, 
            1,
            (80,211,195),
            2)

        text = "Press the button 3 times quickly to end and save..."
        textsize = cv2.getTextSize(text, self.font, 0.6, 1)[0]
        cv2.putText(img,
            text, 
            (int((img.shape[1] - textsize[0])/2),375),
            self.font, 
            0.6,
            (124,160,132),
            1)

        cv2.putText(img,
            'Mikhail Kennerley 2021', 
            (10,500),
            self.font, 
            0.2,
            (144,93,87),
            1)
        cv2.imshow('Frame',img)
        cv2.waitKey(25)



def main(args=None):
    rclpy.init(args=args)

    imu_subscriber = ImuSubscriber()

    rclpy.spin(imu_subscriber)

    imu_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
