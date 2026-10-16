/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习).
 *            The code ADAPTS the corresponding hardware circuit board(智能汽车-ICAR),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file camera.cpp
 * @author HC
 * @brief 图像感知
 * @version 0.1
 * @date 2023/12/14 10:11:42
 * @copyright  :Copyright (c) 2023
 * @note 具体功能模块:
 *                    [01] 绘制田字格作为基准线
 *                    [02] 通过基准线来调整摄像头位置
 */

#include <fstream>            // 文件操作类
#include <iostream>           // 输入输出类
#include <opencv2/opencv.hpp> // OpenCV终端部署
#include <sys/stat.h>         // 获取文件属性
#include "ctrl/predeal.hpp"

using namespace std;
using namespace cv;

#define COLS_IMG 640 // 摄像头：图像的列数
#define ROWS_IMG 480 // 摄像头：图像的行数

int main(int argc, char const *argv[])
{
  // 打开摄像头
  VideoCapture capture("/dev/video0", CAP_V4L2);
  if (!capture.isOpened())
  {
    cout << "can not open video device " << endl;
    return 1;
  }

  capture.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G')); // 强制MJPG编码格式，覆盖默认配置
  capture.set(CAP_PROP_FRAME_WIDTH, COLS_IMG);  // 设置图像的列数
  capture.set(CAP_PROP_FRAME_HEIGHT, ROWS_IMG); // 设置图像的行数
  capture.set(CAP_PROP_BUFFERSIZE, 1);          // 设置缓冲区为1帧，减少延迟

  double rate = capture.get(CAP_PROP_FPS);            // 读取图像的帧率
  double width = capture.get(CAP_PROP_FRAME_WIDTH);   // 读取图像的宽度
  double height = capture.get(CAP_PROP_FRAME_HEIGHT); // 读取图像的高度
  cout << "Camera Param: frame rate = " << rate << " width = " << width
       << " height = " << height << endl;

  // 读取xml中的相机标定参数
  shared_ptr<Predeal> predeal = make_shared<Predeal>(-1); // 图像预处理类

  // 标定采图计数器
  static int index = 0;
  string calibPath = "../res/calibration/temp/";
  struct stat buffer;
  if (stat(calibPath.c_str(), &buffer) != 0) // 判断文件夹是否存在
  {
    string command = "mkdir -p " + calibPath;
    system(command.c_str());
  }

  while (1)
  {
    Mat img;
    if (!capture.read(img))
    {
      cout << "no video frame" << endl;
      continue;
    }
    Mat imgCor = img.clone();

    // 绘制田字格：基准线
    uint16_t rows = ROWS_IMG / 30; // 8
    uint16_t cols = COLS_IMG / 32; // 10

    for (size_t i = 1; i < rows; i++) // 使用for循环绘制行线
    {
      line(img, Point(0, 30 * i), Point(img.cols - 1, 30 * i), Scalar(211, 211, 211), 1);
    }
    for (size_t i = 1; i < cols; i++) // 使用for循环绘制列线
    {
      if (i == (int)(cols / 2))
        line(img, Point(32 * i, 0), Point(32 * i, img.rows - 1), Scalar(0, 0, 255), 2);
      else
        line(img, Point(32 * i, 0), Point(32 * i, img.rows - 1), Scalar(211, 211, 211), 1);
    }

    // 显示已保存图片数量
    putText(img, "Saved: " + to_string(index), Point(10, 30), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 255, 0), 1);
    putText(img, "[Space] Capture", Point(10, 60), FONT_HERSHEY_TRIPLEX, 0.6, Scalar(255, 255, 255), 1);
    imshow("imgOri", img);

    //[02] 图像预处理
    cv::resize(imgCor, imgCor, cv::Size(320, 240), 0, 0, cv::INTER_NEAREST);
    imshow("imgCor", imgCor);

    int key = waitKey(10);
    if (key == 32) // 空格键保存原始图像（用于标定）
    {
      index++;
      string name = calibPath + to_string(index) + ".jpg";
      imwrite(name, img); // 保存原始640x480图像（未矫正）
      cout << "Saved calibration image: " << name << " [" << img.cols << "x" << img.rows << "]" << endl;
    }
  }

  capture.release();
}