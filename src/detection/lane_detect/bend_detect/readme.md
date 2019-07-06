1. lanePixelRange  车道线像素范围
2. minPixelPerlane 每条车道线的最少像素点，低于阈值则认为车道线无效
3. cameraInfo_callback h(camera 安装高度) l0(光心处的物体到camera的水平距离)
4. cv2.fillConvexPoly 填充无关区域
