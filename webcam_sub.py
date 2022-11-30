# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries

import cv2 # OpenCV library
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image, CompressedImage # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import numpy as np
import os
#from stereovision.exceptions import ChessboardNotFoundError

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription1 = self.create_subscription(Image, 
                'image_raw1', 
                self.listener_callback1, 
                10)
    self.subscription2 = self.create_subscription(Image, 
                'image_raw2', 
                self.listener_callback2, 
                10)
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    self.left_depth1 = np.zeros((300,300,3), np.uint8)
    self.right_depth1 = np.zeros((300,300,3), np.uint8)
   
  def listener_callback1(self, data):
    """
    Callback function.
    """
    
 # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image # opencv이미지 ros2로 가져오기 
    frame180_1 = self.br.imgmsg_to_cv2(data)
    frame_1 = cv2.rotate(frame180_1, cv2.ROTATE_180)
    frame_1 = cv2.resize(frame_1, (300, 300))
    
    
    
    '''
    ##############################################################################
    # YOLO 네트워크 불러오기
    YOLO_net = cv2.dnn.readNet("/home/aa/yolov3-tiny.weights","/home/aa/yolov3-tiny.cfg")    
    
    
    classes = []
    with open("/home/aa/yolo.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]
    layer_names = YOLO_net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in YOLO_net.getUnconnectedOutLayers()]

    #while True:
        
    
    
    h, w, c = frame_1.shape

    # YOLO 입력
    blob = cv2.dnn.blobFromImage(frame_1, 0.00392, (416, 416), (0, 0, 0),
    True, crop=False)
    YOLO_net.setInput(blob)
    outs = YOLO_net.forward(output_layers)

    # 각각의 데이터를 저장할 빈 리스트
    class_ids = []
    confidences = []
    boxes = []

    for out in outs:

        for detection in out:

            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > 0.4:
                # Object detected # 탐지된 객체의 너비, 높이 및 중앙 좌표값 찾기
                center_x = int(detection[0] * w)
                center_y = int(detection[1] * h)
                dw = int(detection[2] * w)
                dh = int(detection[3] * h)
                
                # Rectangle coordinate # 객체의 사각형 테두리 중 좌상단 좌표값 찾기
                x = int(center_x - dw / 2)
                y = int(center_y - dh / 2)
                
                boxes.append([x, y, dw, dh])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    # Non Maximum Suppression (겹쳐있는 박스 중 confidence 가 가장 높은 박스를 선택)
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.45, 0.4)

    # 후보 박스 중 선택된 박스의 인덱스 출력
    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            score = f"{confidences[i]:.2f}"

            # 경계상자와 클래스 정보 이미지에 입력
            cv2.rectangle(frame_1, (x, y), (x + w, y + h), (0, 0, 255), 5)
            cv2.putText(frame_1, label, (x, y - 20), cv2.FONT_ITALIC, 0.5, 
            (255, 255, 255), 1)
            #cv2.putText(frame, score, (x + 70, y - 20), cv2.FONT_ITALIC, 0.3, (0, 0, 255), 2)
    
	

    #resize_frame = cv2.resize(frame, (0, 0), fx=2, fy=2, interpolation=cv2.INTER_LINEAR)
    '''
    
    self.right_depth1 = cv2.cvtColor(frame_1, cv2.COLOR_BGR2GRAY)
    cv2.imshow("right", frame_1)
    #right = cv2.resize(self.right_depth1, (300, 300))
    #cv2.imshow("right", right)
    #cv2.imshow("YOLOv3", frame)
    cv2.waitKey(1)
    #self.depthmap()
    
##############################################################################
	

  def listener_callback2(self, data):
    """
    Callback function.
    """
    
 # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image # opencv이미지 ros2로 가져오기 
    frame180_2 = self.br.imgmsg_to_cv2(data)
    frame_2 = cv2.rotate(frame180_2, cv2.ROTATE_180)
    frame_2 = cv2.resize(frame_2, (300, 300))
    
    '''
    ##############################################################################
    # YOLO 네트워크 불러오기
    YOLO_net = cv2.dnn.readNet("/home/aa/yolov3-tiny.weights","/home/aa/yolov3-tiny.cfg")    
    
    
    classes = []
    with open("/home/aa/yolo.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]
    layer_names = YOLO_net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in YOLO_net.getUnconnectedOutLayers()]

    #while True:
        
    
    
    h, w, c = frame_2.shape

    # YOLO 입력
    blob = cv2.dnn.blobFromImage(frame_2, 0.00392, (416, 416), (0, 0, 0),
    True, crop=False)
    YOLO_net.setInput(blob)
    outs = YOLO_net.forward(output_layers)

    # 각각의 데이터를 저장할 빈 리스트
    class_ids = []
    confidences = []
    boxes = []

    for out in outs:

        for detection in out:

            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > 0.01:
                # Object detected # 탐지된 객체의 너비, 높이 및 중앙 좌표값 찾기
                center_x = int(detection[0] * w)
                center_y = int(detection[1] * h)
                dw = int(detection[2] * w)
                dh = int(detection[3] * h)
                
                # Rectangle coordinate # 객체의 사각형 테두리 중 좌상단 좌표값 찾기
                x = int(center_x - dw / 2)
                y = int(center_y - dh / 2)
                
                boxes.append([x, y, dw, dh])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    # Non Maximum Suppression (겹쳐있는 박스 중 confidence 가 가장 높은 박스를 선택)
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.45, 0.4)

    # 후보 박스 중 선택된 박스의 인덱스 출력
    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            score = f"{confidences[i]:.2f}"

            # 경계상자와 클래스 정보 이미지에 입력
            cv2.rectangle(frame_2, (x, y), (x + w, y + h), (0, 0, 255), 5)
            cv2.putText(frame_2, label, (x, y - 20), cv2.FONT_ITALIC, 0.5, 
            (255, 255, 255), 1)
            #cv2.putText(frame, score, (x + 70, y - 20), cv2.FONT_ITALIC, 0.3, (0, 0, 255), 2)
            target_x = (x+w)/2
            target_y = (y+h)/2
    
    #resize_frame = cv2.resize(frame, (0, 0), fx=2, fy=2, interpolation=cv2.INTER_LINEAR)
    '''
    
    self.left_depth1 = cv2.cvtColor(frame_2, cv2.COLOR_BGR2GRAY)
    #cv2.imshow("left", left)
    cv2.imshow("left", frame_2)
    #cv2.imshow("left", left_depth1)
    #cv2.imshow("YOLOv3", frame)
    cv2.waitKey(1)
##############################################################################
  def depthmap(self):
    #stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
    stereo = cv2.StereoBM_create(numDisparities=0, blockSize=21)
    #disparity = stereo.compute(right_depth,left_depth)
	
    right_depth = cv2.resize(self.right_depth1, (300, 300))
    left_depth = cv2.resize(self.left_depth1, (300, 300))
    

    DEPTH_VISUALIZATION_SCALE = 2048
    
    depth = stereo.compute(left_depth, right_depth)
    #plt.imshow(disparity,'gray')
    #plt.show()


    cv2.imshow('asd', depth / DEPTH_VISUALIZATION_SCALE)
    #cv2.waitKey(1)

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  #print(taget_x, target_y)
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
