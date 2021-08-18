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

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/freertos_string_publisher',
            self.listener_callback,
            20)
        self.subscription

        self.mov_rand = movementRandomiser()
        self.recorded_list = []
        self.recorded_label = []
        self.recorded_mag = []
        self.i=0

    def listener_callback(self,msg):
        if self.i > 1000:
            try:
                files = os.listdir('data')
                files = [int(x.split('.')[0]) for x in files]
                listname = 'data/'+str(max(files)+1)
            except:
                listname = 'data/1'
            with open(listname+'.pkl', 'wb') as f:
                pickle.dump([self.recorded_list ,self.recorded_label,self.recorded_mag],f)
            self.get_logger().info('Saved stuff... into: %s' % (listname))
            cv2.destroyWindow('Frame')
            self.destroy_node()
            return
        label,mag = self.mov_rand.start_vid()
        self.recorded_list.append(list(msg.data))
        self.recorded_label.append(label)
        self.recorded_label.append(mag)
        self.get_logger().info('I heard stuff... %i' % (self.i))
        self.i += 1


class movementRandomiser:
    def __init__(self):
        self.movements = ['NONE','UP','DOWN','LEFT','RIGHT']
        self.mov_idx = list(range(len(self.movements)))

        # This holds the last, current and next movement being recorded
        # Required for forecasting as well as a short delay when switching
        self.last_mov = 0
        self.cur_mov = 0
        self.next_mov = random.choice([x for x in self.mov_idx if x != self.cur_mov])

        # Time variables for changing as well as delays
        self.rand_time = 4
        self.check_time = time.time()
        self.check_time_delay = 0

        # Holds the movement magnitute
        self.movement_mag = 0 #random.randint(1,5)/5
        self.next_movement_mag = random.randint(1,5)/5
        self.check_mag_time = time.time()
        self.rand_mag_time = random.randint(10,40)/10

        self.font = cv2.FONT_HERSHEY_SIMPLEX

    def start_vid(self):
        time_left = (self.check_time + self.rand_time) - time.time()

        # Runs when rand_time limit ends, creates new rand_time and passes movement queue
        if time_left<0:
            self.check_time = time.time()
            self.check_mag_time = time.time()
            self.rand_time = random.randint(3000,7000)/1000
            self.last_mov = self.cur_mov
            self.cur_mov = self.next_mov
            self.next_mov = random.choice([x for x in self.mov_idx if x != self.cur_mov])
            self.check_time_delay = time.time() + 0.3
            self.movement_mag = self.next_movement_mag
            self.next_movement_mag = random.randint(1,5)/5
            if self.cur_mov == 0:
                self.movement_mag = 0

        if time.time()-self.check_mag_time > self.rand_mag_time and self.cur_mov and time_left > 1.5:
            self.check_mag_time = time.time()
            self.movement_mag = self.next_movement_mag
            self.next_movement_mag = random.randint(1,5)/5
            self.rand_mag_time = random.randint(10,30)/10

        # Image for user
        img = np.zeros((512, 512, 3), dtype = "uint8")
        # Current Movement
        textsize = cv2.getTextSize(self.movements[self.cur_mov], self.font, 3, 6)[0]
        cv2.putText(img,
            self.movements[self.cur_mov], 
            (int((img.shape[1] - textsize[0])/2),130),
            self.font, 
            3,
            (255,255,255),
            6)
        # Current Magnitute
        textsize = cv2.getTextSize(str(self.movement_mag), self.font, 3, 6)[0]
        cv2.putText(img,
            str(self.movement_mag), 
            (int((img.shape[1] - textsize[0])/2),200),
            self.font, 
            3,
            (255,255,255),
            6)
        #Time LEft
        textsize = cv2.getTextSize(str(round(time_left,1)), self.font, 1.5, 3)[0]
        cv2.putText(img,
            str(round(time_left,1)), 
            (int((img.shape[1] - textsize[0])/2),300),
            self.font, 
            1.5,
            (0,255,0),
            3)

        if time_left>1.5:
            clr = (0,100,100)
        else:
            clr = (0,255,255)

        if time_left<=1.5:
            clr2 = (0,255,255)
        else:
            clr2 = (0,255,0)

        # NExt Movement
        textsize = cv2.getTextSize(self.movements[self.next_mov], self.font, 2, 3)[0]
        cv2.putText(img,
            self.movements[self.next_mov], 
            (int((img.shape[1] - textsize[0])/2),400),
            self.font, 
            2,
            clr,
            3)
        # Current Magnitute
        textsize = cv2.getTextSize(str(self.next_movement_mag), self.font, 2, 3)[0]
        cv2.putText(img,
            str(self.next_movement_mag), 
            (int((img.shape[1] - textsize[0])/2),450),
            self.font, 
            2,
            clr2,
            3)
        cv2.imshow('Frame',img)
        cv2.waitKey(25)


        # Delay in the recorded movement to account for reaction time
        if time.time() > self.check_time_delay:
            ret_mov = self.cur_mov
        else:
            ret_mov = self.last_mov

        return ret_mov, self.movement_mag



def main(args=None):
    rclpy.init(args=args)

    imu_subscriber = ImuSubscriber()

    rclpy.spin(imu_subscriber)

    imu_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
