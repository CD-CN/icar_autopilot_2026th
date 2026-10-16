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
 * @file predeal.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 图像预处理
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "ctrl/predeal.hpp"

using namespace cv;
using namespace std;

/**
 * @brief 图像矫正参数初始化
 *
 * @param bin 二值化阈值
 */
Predeal::Predeal(int bin)
    : binary(bin)
{
    cout << "图像预处理初始化完成（无去畸变）" << endl;
}

/**
 * @brief 图像二值化
 *
 * @param img  输入图像
 * @return cv::Mat 二值化图像
 */
cv::Mat Predeal::binaryzation(cv::Mat &img)
{
    cv::Mat imgGray, imgBin;
    cvtColor(img, imgGray, COLOR_BGR2GRAY); // RGB转灰度图

    if (binary < 0)
        threshold(imgGray, imgBin, 0, 255, THRESH_OTSU); //  采用大津法二值化
    else
    {
        if (binary > 255)
            binary = 255;
        threshold(imgGray, imgBin, binary, 255, THRESH_BINARY); // 固定阈值方法
    }

    // 中值滤波降噪（消除孤立噪点）
    if (medianSize > 0)
        medianBlur(imgBin, imgBin, medianSize);

    // 图像转换
    Mat imgInv = Mat::zeros(imgBin.size(), imgBin.type());
    bitwise_not(imgBin, imgInv);

    return imgInv;
}

/**
 * @brief 图像裁剪
 *
 * @param img
 */
void Predeal::imgCutting(cv::Mat &img)
{
    // 图像裁剪
    // 提取 55~175 行，列方向为 30~310
    cv::Rect roi(40, 55, 260, 120); // (x, y, width, height)
    img = img(roi);                 // 抠图

    // 纵向拉伸到300x180（height=300, width=180）
    cv::resize(img, img, cv::Size(300, 180), 0, 0, cv::INTER_NEAREST);
}
