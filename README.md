# turtlebot3_opencv

# 11/30

1. 기존 opencv_cam 코드의 img_raw1, img_raw2가 동일한 이미지로 나와 opencv_cam 을 opencv_cam1, opencv_cam2로 나눔. 이후 subscriber로 img_raw1,2를 받아 화면에 두 영상을 띠움

2. yolo 넣어 적용 실패 --> TypeError: only integer scalar arrays can be converted to a scalar index 오류가 output_layers = [layer_names[i - 1] for i in YOLO_net.getUnconnectedOutLayers()]에서 발생함, [layer_names[i[0] - 1]을 수정하여 해결.

3. left, right 카메라로 depthmap 예제를 활용해 거리에 따라 차이를 나타냄
