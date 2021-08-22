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
        self.i=0

    def listener_callback(self,msg):
        if self.i > 1000:
            try:
                files = os.listdir('data/simple')
                files = [int(x.split('.')[0]) for x in files]
                listname = 'data/simple/'+str(max(files)+1)
            except:
                listname = 'data/simple/1'
            with open(listname+'.pkl', 'wb') as f:
                pickle.dump([self.recorded_list ,self.recorded_label],f)
            self.get_logger().info('Saved stuff...%s' % (listname))
            cv2.destroyWindow('Frame')
            self.destroy_node()
            exit()
        label = self.mov_rand.start_vid()
        self.recorded_list.append(list(msg.data))
        self.recorded_label.append(label)
        self.get_logger().info('I heard stuff... %i' % (self.i))
        self.i += 1


class movementRandomiser:
    def __init__(self):
        self.movements = ['None','Forward','Backward','Left','Right','Clockwise','C-Clockwise']
        self.mov_idx = list(range(len(self.movements)))
        self.movement_max = 1

        # This holds the current movement being recorded
        self.last_mov = 0
        self.cur_mov = 0
        self.next_mov = random.choice([x for x in self.mov_idx if x != self.cur_mov])
        self.rand_time = 4
        self.check_time = time.time()
        self.check_time_delay = 0

        self.font = cv2.FONT_HERSHEY_SIMPLEX

    def start_vid(self):
        time_left = (self.check_time + self.rand_time) - time.time()

        if time_left<0:
            self.check_time = time.time()
            self.rand_time = random.randint(3000,7000)/1000
            self.last_mov = self.cur_mov
            self.cur_mov = self.next_mov
            self.next_mov = random.choice([x for x in self.mov_idx if x != self.cur_mov])
            self.check_time_delay = time.time() + 0.3

        img = np.zeros((512, 512, 3), dtype = "uint8")

        textsize = cv2.getTextSize(self.movements[self.cur_mov], self.font, 3, 6)[0]
        cv2.putText(img,
            self.movements[self.cur_mov], 
            (int((img.shape[1] - textsize[0])/2),150),
            self.font, 
            3,
            (255,255,255),
            6)


        textsize = cv2.getTextSize(str(round(time_left,1)), self.font, 1.5, 3)[0]
        cv2.putText(img,
            str(round(time_left,1)), 
            (int((img.shape[1] - textsize[0])/2),300),
            self.font, 
            1.5,
            (0,255,0),
            3)

        textsize = cv2.getTextSize(self.movements[self.next_mov], self.font, 2, 3)[0]
        cv2.putText(img,
            self.movements[self.next_mov], 
            (int((img.shape[1] - textsize[0])/2),400),
            self.font, 
            2,
            (0,255,255),
            3)
        cv2.imshow('Frame',img)
        cv2.waitKey(25)

        if time.time() > self.check_time_delay:
            ret_mov = self.cur_mov
        else:
            ret_mov = self.last_mov

        return ret_mov



def main(args=None):
    rclpy.init(args=args)

    imu_subscriber = ImuSubscriber()

    rclpy.spin(imu_subscriber)

    imu_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
