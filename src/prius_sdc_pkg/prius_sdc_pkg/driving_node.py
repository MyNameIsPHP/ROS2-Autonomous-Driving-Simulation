import cv2
import time
import os

from geometry_msgs.msg import Twist
from rclpy.node import Node 
from cv_bridge import CvBridge 
import rclpy 
from sensor_msgs.msg import Image 
import multiprocessing
from multiprocessing import Process, Queue

from .lane_line_detection import calculate_control_signal
from .traffic_sign_detection import detect_traffic_signs
from .utils.carcontroler import CarController
from .utils.param import Param
from .traffsign.traffic_sign_detection import SignDetector
from .object.object_detection import ObjectDetector



param = Param()

class Car_driver(Node):
    def __init__(self):

        super().__init__('driving_node')
        self.subscriber = self.create_subscription(Image,'/camera/image_raw',self.process_image,10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        
        self.velocity=Twist()
        self.bridge   = CvBridge() # converting ros images to opencv data

        self.g_image_queue = Queue(maxsize=5)
        self.carcontrol = CarController(param)

        self.manager = multiprocessing.Manager()
        self.g_image_queue = Queue(maxsize=5)
        self.signs = self.manager.list()
        self.objects = self.manager.list()
        self.steering_angle_data = self.manager.Value("d",0.0)
        self.throttle_data = self.manager.Value("d",0.0)
        self.p = Process(target=self.process_traffic_sign_loop, args=(self.g_image_queue, self.signs))
        self.o = Process(target=self.process_object_loop, args=(self.g_image_queue, self.objects))
        self.s = Process(target=self.save_image_to_dataset, args=(self.g_image_queue, self.steering_angle_data, self.throttle_data))
        self.p.start()
        self.o.start()
        
    def process_traffic_sign_loop(self, g_image_queue, signs):
        countSign = 0
        lastSign = ''
        detect = SignDetector(param.traffic_sign_model)
        while True:
            # print("process_traffic_sign_loop")

            if g_image_queue.empty():
                time.sleep(0.1)
                continue
            image = g_image_queue.get()
            # Prepare visualization image
            draw = image.copy()
            # Detect traffic signs
            detected_signs = detect.detect_traffic_signs(image, draw=draw)
            if lastSign == '' or lastSign == detected_signs:
                countSign += 1
            else:
                countSign ==0
            
            # Update the shared signs list
            if countSign >= 15:
                signs[:] = detected_signs
            else:
                signs[:] = []
            # Show the result to a window
            # cv2.imshow("Traffic signs", draw)
            # cv2.waitKey(1)
    
    def process_object_loop(self, g_image_queue, objects):
        print("process_object_loop")
        detect = ObjectDetector(param.cascade, param.onnx_session)
        while True:
            # print("process_object_loop")

            if g_image_queue.empty():
                time.sleep(0.1)
                continue
            image = g_image_queue.get()
            
            draw = image.copy()
            
            detected_objects = detect.detect_object(image, draw= draw)

            # Update the shared signs list
            objects[:] = detected_objects
            
            """ DRAW FRAME """
            # cv2.imshow('Detection object', draw)
            # cv2.waitKey(1)

    def save_image_to_dataset(self, g_image_queue, steering_angle_data, throttle_data):
        while True:
            if g_image_queue.empty():
                time.sleep(0.1)
                continue
            dataset_dir = "dataset"

            # Create the "dataset" folder if it doesn't exist
            if not os.path.exists(dataset_dir):
                os.makedirs(dataset_dir)

            image = self.g_image_queue.get()
            # Create a unique name for the image based on the current time
            timestamp = int(time.time() * 1000)
            image_filename = f"{timestamp}_steer_{steering_angle_data.value}_throttle_{throttle_data.value}.png"
            image_path = os.path.join(dataset_dir, image_filename)
            # Save the image to the dataset folder
            print(f"Saved image: {image_filename}")
            cv2.imwrite(image_path, image)
            cv2.waitKey(1)

    def send_cmd_vel(self):
        # self.velocity.linear.x = 2.0        
        # self.velocity.angular.z = 0.5
        self.publisher.publish(self.velocity)

   
    def process_image(self, data): 

      image = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion
    #   image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
      image = cv2.resize(image, (640, 480))
      draw = image.copy()

      # Send back throttle and steering angle
      throttle, steering_angle = self.carcontrol.decision_control(image, self.signs, self.objects, draw = draw)
    #   throttle, steering_angle = calculate_control_signal(image, draw=draw)

       # Update image to g_image_queue - used to run sign detection
      if not self.g_image_queue.full():
          self.g_image_queue.put(image)


      # Angle,Speed,img = self.Car.drive_car(frame)
      
      
      self.velocity.angular.z = float(steering_angle)
      self.velocity.linear.x = float(throttle)      
      print("steering_angle", self.velocity.angular.z)
      print("throttle",  self.velocity.linear.x)

      cv2.imshow("Result", draw)
      cv2.imshow("Image",image)
      cv2.waitKey(1)
        
        
    
def main(args=None):
    
    # s.start()

    rclpy.init(args=args)
    cmd_vel_publisher = Car_driver()
    rclpy.spin(cmd_vel_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
	main()