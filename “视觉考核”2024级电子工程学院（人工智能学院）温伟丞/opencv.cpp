#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;
using namespace chrono;


////////////////////////////  装甲板识别考核题  //////////////////////////////
////这段用了AI来对装甲板中心点位置进行预测，以实现更平滑的位置跟踪
//Point predictPosition(const vector<Point>& previousPositions) {
//    if (previousPositions.empty()) {
//        return Point(0, 0);
//    }
//    Point predicted(0, 0);
//    for (const auto& p : previousPositions) {
//        predicted += p;
//    }
//    return Point(predicted.x / previousPositions.size(), predicted.y / previousPositions.size());
//}
//
//int main() {
//    VideoCapture cap("D:/杂物、文件/装甲板 - 红.mp4");
//    if (!cap.isOpened()) {
//        cout << "无法打开视频！!" << endl;
//        return -1;
//    }
//
//    vector<Point> previousPositions;
//    const int historySize = 10;
//
//    while (true) {
//        Mat frame;
//        cap >> frame;
//
//        if (frame.empty()) {
//            cout << "End of video or can't read the frame!" << endl;
//            break;
//        }
//
//        // 高斯模糊以减少噪声
//        GaussianBlur(frame, frame, Size(5, 5), 0);
//        Mat hsv;
//        cvtColor(frame, hsv, COLOR_BGR2HSV);
//
//        // 定义颜色范围
//        Scalar lower_red1(0, 100, 100);
//        Scalar upper_red1(10, 255, 255);
//        Scalar lower_red2(160, 100, 100);
//        Scalar upper_red2(180, 255, 255);
//        Scalar lower_blue(100, 100, 100);
//        Scalar upper_blue(130, 255, 255);
//
//        // 生成掩膜
//        Mat mask_red1, mask_red2, mask_blue;
//        inRange(hsv, lower_red1, upper_red1, mask_red1);
//        inRange(hsv, lower_red2, upper_red2, mask_red2);
//        inRange(hsv, lower_blue, upper_blue, mask_blue);
//        Mat mask_combined;
//        add(mask_red1, mask_red2, mask_combined);
//        add(mask_combined, mask_blue, mask_combined);
//
//        // 侵蚀和扩大以排除小噪点
//        erode(mask_combined, mask_combined, Mat(), Point(-1, -1), 2);
//        dilate(mask_combined, mask_combined, Mat(), Point(-1, -1), 2);
//        //找出轮廓
//        vector<vector<Point>> contours;
//        findContours(mask_combined, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//        //过滤后轮廓
//        vector<vector<Point>> filteredContours;
//        //设置最小面积减少其他灯光对于结果的干扰
//        const double minArea = 500; 
//        for (const auto& contour : contours) {
//            if (contourArea(contour) > minArea) {
//                filteredContours.push_back(contour);
//            }
//        }
//        //保证找出两条灯条后开始计算
//        if (filteredContours.size() >= 2) {
//            // 计算灯条的边界矩形
//            Rect rect1 = boundingRect(filteredContours[0]);
//            Rect rect2 = boundingRect(filteredContours[1]);
//            //找出灯条矩形中心点
//            Point center1 = (rect1.tl() + rect1.br()) / 2;
//            Point center2 = (rect2.tl() + rect2.br()) / 2;
//
//            // 计算中间区域的矩形框
//            int left = min(rect1.x, rect2.x);
//            int top = min(rect1.y, rect2.y);
//            int right = max(rect1.x + rect1.width, rect2.x + rect2.width);
//            int bottom = max(rect1.y + rect1.height, rect2.y + rect2.height);
//            Rect combinedRect(left, top, right - left, bottom - top);
//
//            // 绘制矩形框
//            rectangle(frame, combinedRect, Scalar(255, 0, 0), 2); // 中间区域框
//            rectangle(frame, rect1, Scalar(0, 255, 0), 2); // 第一个灯条框
//            rectangle(frame, rect2, Scalar(0, 255, 0), 2); // 第二个灯条框
//            //找出装甲板中心点
//            previousPositions.push_back((center1 + center2) / 2);
//            if (previousPositions.size() > historySize) {
//                previousPositions.erase(previousPositions.begin());
//            }
//
//            // 绘制中心点
//            Point armorCenter = (center1 + center2) / 2;
//            circle(frame, armorCenter, 3, Scalar(0, 255, 0), -1);
//        }
//
//        imshow("Detected Armor Plates", frame);
//        if (waitKey(1) == 'q') {
//            break;
//        }
//    }
//
//    cap.release();
//    destroyAllWindows();
//    return 0;
//}




/////////////////  opencv基础第一题  ////////////////
//int main() {
//    // 读取原图
//    Mat image = imread("D:/杂物、文件/opencv基础第一题.png");
//    if (image.empty()) {
//        std::cerr << "打开图像失败!" << std::endl;
//        return -1;
//    }
//
//    // 色彩分割（提取红色区域）
//    Mat hsv_image, mask;
//    cvtColor(image, hsv_image, COLOR_BGR2HSV);
//
//    // 定义红色范围
//    Scalar lower_red1(0, 100, 100);   // 红色的下阈值
//    Scalar upper_red1(10, 255, 255);  // 红色的上阈值
//    Scalar lower_red2(160, 100, 100); // 红色的下阈值（高范围）
//    Scalar upper_red2(180, 255, 255); // 红色的上阈值（高范围）
//
//    // 通过掩膜将图像二值化
//    Mat mask1, mask2;
//    inRange(hsv_image, lower_red1, upper_red1, mask1);
//    inRange(hsv_image, lower_red2, upper_red2, mask2);
//    mask = mask1 | mask2;
//    Mat binary_image;
//    mask.convertTo(binary_image, CV_8U);
//
//    // 边缘提取
//    Mat edges;
//    Canny(binary_image, edges, 100, 200);
//
//    // 创建一个空的黑色背景
//    Mat edge_image = Mat::zeros(image.size(), CV_8UC3);
//
//    // 将边缘绘制为蓝色
//    Mat colored_edges;
//    cvtColor(edges, colored_edges, COLOR_GRAY2BGR);
//    colored_edges.setTo(Scalar(255, 0, 0), edges);
//
//    // 显示原图
//    namedWindow("Original Image", WINDOW_NORMAL);
//    resizeWindow("Original Image", 1280, 720);
//    imshow("Original Image", image);
//    // 显示黑白图
//    namedWindow("Binary Image", WINDOW_NORMAL);
//    resizeWindow("Binary Image", 1280, 720);
//    imshow("Binary Image", binary_image);
//    //  显示蓝色边缘图像
//    namedWindow("Edges", WINDOW_NORMAL);
//    resizeWindow("Edges", 1280, 720);
//    imshow("Edges", colored_edges);
//
//    // 保存蓝色边缘图像
//    imwrite("edges_image.png", colored_edges);
//
//    waitKey(0);
//    return 0;
//}


/////////////////  opencv基础第三题  ////////////////
//
//Rect selection; //选中的矩形区域
//Point origin;  //鼠标按下起始点
//bool selecting = false;  //用于判断鼠标书否按下
//
//     //鼠标回调函数
//void onMouse(int event, int x, int y, int flags, void* param) {
//    Mat* img = (Mat*)param;  //将param指针转为指向img
//    //鼠标点击检测
//    if (event == EVENT_LBUTTONDOWN) {
//        origin = Point(x, y);
//        selection = Rect(x, y, 0, 0);
//        selecting = true;
//    }//鼠标拖动检测
//    else if (event == EVENT_MOUSEMOVE) {
//        if (selecting) {
//            selection.width = x - origin.x;
//            selection.height = y - origin.y;
//            Mat temp = img->clone(); //克隆当前图像
//            rectangle(temp, selection, Scalar(255, 0, 0), 1);
//            imshow("Image", temp);
//
//            // 显示鼠标当前坐标和RGB值
//            if (x >= 0 && y >= 0 && x < img->cols && y < img->rows) {
//                Vec3b pixel = img->at<Vec3b>(Point(x, y));
//                cout << "RGB: (" << (int)pixel[2] << ", " << (int)pixel[1] << ", " << (int)pixel[0] << ") "
//                    << "坐标: (" << x << ", " << y << ")\r";
//                cout.flush();
//            }
//        }
//    }//鼠标松开检测
//    else if (event == EVENT_LBUTTONUP) {
//        selecting = false;
//        // 在从右往左或从下往上选取时将矩形翻转以保证矩形坐标和宽高为正值 
//        if (selection.width < 0) {
//            selection.x += selection.width;//调整左上角x坐标
//            selection.width *= -1; //宽度取绝对值
//        }
//        if (selection.height < 0) {
//            selection.y += selection.height; // 调整左上角y坐标
//            selection.height *= -1; // 高度取绝对值
//        }
//
//        // 裁剪
//        Mat cropped = (*img)(selection);
//        imshow("Cropped Image", cropped);
//
//        // 输出框中心像素点坐标
//        Point center = Point(selection.x + selection.width / 2, selection.y + selection.height / 2);
//        cout << "框中心像素点坐标: (" << center.x << ", " << center.y << ")" << endl;
//    }
//}
//
//int main() {
//    Mat img = imread("D:/杂物、文件/opencv基础第三题.png");
//    if (img.empty()) {
//        cout << "打开图像失败!" << endl;
//        return -1;
//    }
//
//    namedWindow("Image");
//    setMouseCallback("Image", onMouse, &img);
//    imshow("Image", img);
//
//    waitKey(0);
//    return 0;
//}



/////////////////  opencv基础第二题（没来得及打印标定板）  ////////////////
//
//int main() {
//    VideoCapture cap(0);
//    if (!cap.isOpened()) {
//        cout << "无法打开摄像头！" << endl;
//        return -1;
//    }
//
//    // 获取视频的宽度高度和帧率
//    int width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));
//    int height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));
//    int fps = static_cast<int>(cap.get(CAP_PROP_FPS));
//
//    cout << "视频图像大小: " << width << "x" << height << endl;
//    cout << "帧率 (FPS): " << fps << endl;
// 
//    namedWindow("Camera", WINDOW_AUTOSIZE);
//
//    // 创建滑动条
//    int exposure = 0;
//    int brightness = 0;
//    createTrackbar("曝光时间", "Camera", &exposure, 100);
//    createTrackbar("亮度", "Camera", &brightness, 100);
//
//    // 视频录制设置
//    VideoWriter writer("output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, Size(width, height));
//    if (!writer.isOpened()) {
//        cout << "无法打开视频写入！" << endl;
//        return -1;
//    }
//
//    Mat frame;
//    while (true) {
//        cap >> frame;
//        if (frame.empty()) break;
//        // 设置曝光和亮度
//        cap.set(CAP_PROP_EXPOSURE, exposure / 10.0);
//        frame.convertTo(frame, -1, 1, brightness);
//        imshow("Camera", frame);
//        writer << frame;
//        if (waitKey(30) >= 0) break;
//    }
//
//    cap.release();
//    writer.release();
//    destroyAllWindows();
//
//    return 0;
//}


/////////////////  opencv应用第一题  ////////////////

//int main() {
//    Mat image = imread("D:/杂物、文件/opencv应用第一题.png");
//    if (image.empty()) {
//        cout << "Could not open or find the image!" << endl;
//        return -1;
//    }
//
//    //变换为HSV
//    Mat hsvImage;
//    cvtColor(image, hsvImage, COLOR_BGR2HSV);
//
//    // 创建红色掩膜
//    Scalar lowerRed(0, 100, 100);
//    Scalar upperRed(10, 255, 255);
//    Mat mask1;
//    inRange(hsvImage, lowerRed, upperRed, mask1);
//    lowerRed = Scalar(160, 100, 100);
//    upperRed = Scalar(180, 255, 255);
//    Mat mask2;
//    inRange(hsvImage, lowerRed, upperRed, mask2);
//    Mat mask = mask1 | mask2;
//    //查找轮廓
//    vector<vector<Point>> contours;
//    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//
//    if (!contours.empty()) {
//        // 找到最大的轮廓
//        auto largestContour = max_element(contours.begin(), contours.end(), [](const vector<Point>& a, const vector<Point>& b) {
//            return contourArea(a) < contourArea(b);
//            });
//        //绘制轮廓  
//        drawContours(image, contours, distance(contours.begin(), largestContour), Scalar(255, 0, 0), 2);  
//        //绘制矩形
//        Rect boundingBox = boundingRect(*largestContour);
//        rectangle(image, boundingBox, Scalar(0, 255, 0), 2);
//    }
//
//    imshow("Apple", image);
//    waitKey(0);
//    return 0;
//}




/////////////////// 赛事题第二题 ////////////////////
//
//// 计算轮廓的中心点
//Point2f findCenter(const vector<Point>& points) {
//    Moments m = moments(points, true);
//    return Point2f(m.m10 / m.m00, m.m01 / m.m00);
//}
//
//// 计算旋转方向（中文显示会显示为问号）
//string determineRotationDirection(const Point2f& prev, const Point2f& curr) {
//    // 判断当前中心点和上一个中心点的y坐标，确定旋转方向
//    if (curr.y < prev.y) {
//        return "Clockwise";
//    }
//    else {
//        return "Counterclockwise";
//    }
//}
//
//int main() {
//    VideoCapture capture("D:/杂物、文件/能量机关-红.mp4");
//    if (!capture.isOpened()) { 
//        cout << "打开视频失败" << endl;
//        return -1; 
//    }
//
//
//    Mat frame; // 用于存储每一帧图像
//    Point2f prevCenter; // 上一帧的中心点
//    bool firstFrame = true; // 标记是否为第一帧
//    string rotationDirection; // 存储旋转方向
//    bool directionDetermined = false; // 是否已确定旋转方向
//
//
//    while (true) {
//        capture >> frame;
//        if (frame.empty()) { 
//            break;
//        }
//        //图像处理（灰度、高斯模糊、边缘检测）
//        Mat gray, blurred, edged;
//        cvtColor(frame, gray, COLOR_BGR2GRAY);
//        GaussianBlur(gray, blurred, Size(5, 5), 0);
//        Canny(blurred, edged, 50, 150);
//
//        vector<vector<Point>> contours; // 用vector来存储检测到的轮廓（多个点）
//        findContours(edged, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); // 查找轮廓
//
//
//        for (const auto& contour : contours) {
//            if (contourArea(contour) > 500) { // 过滤掉面积小于500的轮廓（排除干扰）
//                Point2f currCenter = findCenter(contour); // 获取当前轮廓的中心点
//                drawContours(frame, contours, -1, Scalar(0, 255, 0), 2); // 绘制轮廓
//                circle(frame, currCenter, 5, Scalar(255, 0, 0), -1); // 在中心点绘制圆圈
//
//                // 判断旋转方向
//                if (!firstFrame && !directionDetermined) { // 当非第一帧且方向未确定
//                    rotationDirection = determineRotationDirection(prevCenter, currCenter); // 确定旋转方向
//                    directionDetermined = true; // 标记方向已确定
//                }
//                // 防止开头误判
//                else if (firstFrame) { // 如果是第一帧
//                    firstFrame = false; // 标记为非第一帧
//                }
//                // 更新上一帧的中心点
//                prevCenter = currCenter;
//            }
//        }
//
//        // 显示旋转方向
//        if (directionDetermined) { 
//            putText(frame, "Rotation Direction: " + rotationDirection, Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2);//中文通过putText显示会全是问号，所以改成英文了。
//        }
//
//        imshow("Recognition Result", frame);
//        if (waitKey(1) >= 0) break;
//    }
//
//    capture.release();
//    return 0;
//}