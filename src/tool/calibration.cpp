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
 * @file calibration.cpp
 * @author Leo
 * @brief 基于OpenCV的相机标定
 * @version 0.1
 * @date 2022-02-23
 * @copyright Copyright (c) 2022
 * @note 采用标准棋盘格进行标定，标定流程如下：
 *      1. 拍摄标定图片（遥控手柄采图/collection.cpp）
 *      2. 提取每张图片的角点信息
 *      3. 提取图像的亚像素角点信息
 *      4. 在棋盘图上绘制内角点（检验标定结果）
 *      5. 开始标定
 *      6. 评价标定参数
 *      7. 矫正图像
 */

#include <fstream>
#include <iostream>
#include <numeric>
#include <algorithm>
#include <random>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <sys/stat.h>
#include "utils/tools.hpp"
#include "utils/show.hpp"

using namespace cv;
using namespace std;

Size sizeBoard = Size(11, 8);   // 标定板的角点数(行,列)
Size sizeSquare = Size(20, 20); // 实际测量标定板上每个棋盘格的边长（长,宽）：单位mm
const double VALIDATION_RATIO = 0.2; // 验证集占比20%
const int RANDOM_SEED = 42;          // 固定随机种子，保证可复现
string getFilename(string sourcePath);

/**
 * @brief 计算某一样本集的RMSE（均方根误差）
 *
 * @param objectPoints  3D角点集（每张图一组）
 * @param imagePoints   2D角点集（每张图一组）
 * @param rvecs         旋转向量集（来自calibrateCamera或solvePnP）
 * @param tvecs         平移向量集
 * @param cameraMatrix  相机内参矩阵
 * @param distCoeffs    畸变系数
 * @param imagesPath    图片路径（用于输出每张图误差）
 * @param fout          输出文件流
 * @param setLabel      "训练集" / "验证集"
 * @return double 整体RMSE（像素）
 */
double computeRMSE(const vector<vector<Point3f>> &objectPoints,
                   const vector<vector<Point2f>> &imagePoints,
                   const vector<Mat> &rvecs, const vector<Mat> &tvecs,
                   const Mat &cameraMatrix, const Mat &distCoeffs,
                   const vector<String> &imagesPath,
                   ofstream &fout, const string &setLabel)
{
    if (objectPoints.empty() || imagePoints.empty())
        return 0.0;

    fout << "【" << setLabel << "】每幅图像的RMSE:" << endl;
    cout << "【" << setLabel << "】每幅图像的RMSE:" << endl;

    double sumSquaredError = 0.0; // 所有角点的误差平方和
    int totalPoints = 0;          // 所有角点总数

    for (size_t i = 0; i < imagePoints.size(); i++)
    {
        vector<Point2f> projectedPoints;
        projectPoints(objectPoints[i], rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, projectedPoints);

        double sumSqErrImage = 0.0; // 该图像的误差平方和
        for (size_t j = 0; j < imagePoints[i].size(); j++)
        {
            double dx = projectedPoints[j].x - imagePoints[i][j].x;
            double dy = projectedPoints[j].y - imagePoints[i][j].y;
            sumSqErrImage += dx * dx + dy * dy;
        }
        sumSquaredError += sumSqErrImage;
        totalPoints += imagePoints[i].size();

        double rmseImage = sqrt(sumSqErrImage / imagePoints[i].size());
        string filename = getFilename(imagesPath[i]);
        cout << "  图像[" << filename << "]的RMSE: " << rmseImage << " 像素" << endl;
        fout << "  图像[" << filename << "]的RMSE: " << rmseImage << " 像素" << endl;
    }

    double rmseTotal = sqrt(sumSquaredError / totalPoints);
    cout << setLabel << "整体RMSE: " << rmseTotal << " 像素 (" << imagePoints.size() << "张)" << endl;
    fout << setLabel << "整体RMSE: " << rmseTotal << " 像素 (" << imagePoints.size() << "张)" << endl
         << endl;
    return rmseTotal;
}

/**
 * @brief 相机标定程序
 * @note 输出标定参数文件.yml 和 评价结果.txt
 * @param imagesFolder 标定板图像路径（文件夹）
 */
int main(int argc, char *argv[])
{
    string imagesFolder = "../res/calibration/temp/";       // 打开标定图像
    Size sizeImage;                                         // 图像的尺寸
    Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 摄像机内参矩阵
    Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));   // 相机的畸变矩阵
    vector<String> imagesPath;
    Show show(2);                          // 初始化UI显示窗口
    glob(imagesFolder, imagesPath, false); // OpenCV提取文件夹中的所有文件
    struct stat buffer;

    //[1] 提取每张图片的角点信息
    cout << "[1] 提取每张图片的角点信息" << endl;
    vector<vector<Point2f>> pointsCorners; // 保存检测到的所有角点
    bool getImageSize = false;
    for (int i = 0; i < imagesPath.size(); i++)
    {
        Mat imageInput = imread(imagesPath[i]); // OpenCV读取图像
        if (!getImageSize)                      // 初始化图像Size信息
        {
            sizeImage.width = imageInput.cols;
            sizeImage.height = imageInput.rows;
            getImageSize = true;
            cout << "Image.width = " << sizeImage.width << "  |  Image.height = " << sizeImage.height << endl;
        }

        vector<Point2f> pointCorners;                                   // 每幅图像上检测到的角点坐标
        if (findChessboardCorners(imageInput, sizeBoard, pointCorners)) // 寻找图像角点
        {
            Mat imageGray;
            cvtColor(imageInput, imageGray, CV_RGB2GRAY);
            //[2] 亚像素精确化
            find4QuadCornerSubpix(imageGray, pointCorners, Size(5, 5)); // 对粗提取的角点信息进一步优化
            // cornerSubPix(imageGray,pointCorners,Size(5,5),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));

            drawChessboardCorners(imageInput, sizeBoard, pointCorners, true); // 在标定板图像上绘制已识别的角点信息

            std::string windowName = "Corners";
            cv::namedWindow(windowName, WINDOW_NORMAL); // 图像名称
            imshow("Corners", imageInput);              //[3] 在原始图形中绘制角点信息并展示

            // 存储角点绘制图像
            string filename = getFilename(imagesPath[i]);
            string img_path = "../res/calibration/corners/" + filename;

            if (stat(img_path.c_str(), &buffer) != 0) // 判断文件夹是否存在
            {
                string command;
                command = "mkdir -p ../res/calibration/corners/";
                system(command.c_str()); // 利用os创建文件夹
            }

            imwrite(img_path, imageInput);

            uint16_t countCorner = sizeBoard.width * sizeBoard.height; // 每张图像上该有的角点数量
            if (pointCorners.size() >= countCorner)
            {
                pointsCorners.push_back(pointCorners); // 保存亚像素角点
            }

            cout << imagesPath[i] << " 样片合格!" << endl;
            waitKey(500); // 停顿500ms
        }
        else // 图像不存在角点
        {
            cout << "Can not find chessboard corners! Path:" << imagesFolder << endl;
            cout << imagesPath[i] << " 样片不合格!!!" << endl;
            cout << imagesPath[i] << " 请删除该照片!!!" << endl;
            exit(0); // 直接退出标定：否者后续文件名无法对应
        }
    }
    if (imagesPath.size() < 1)
    {
        cout << "Can't find any images!!" << endl;
        return false;
    }
    if (pointsCorners.size() < 1)
    {
        cout << "Can't find any images!!" << endl;
        return false;
    }

    cout << "已筛选 " << pointsCorners.size() << "张合格照片进行标定 | Loading...." << endl;

    // [新增] 分割训练集/验证集（80/20，随机可复现）
    vector<int> indices(pointsCorners.size());
    iota(indices.begin(), indices.end(), 0);
    mt19937 rng(RANDOM_SEED);
    shuffle(indices.begin(), indices.end(), rng);

    int valCount = max(1, (int)(pointsCorners.size() * VALIDATION_RATIO));
    vector<int> valIdx(indices.begin(), indices.begin() + valCount);
    vector<int> trainIdx(indices.begin() + valCount, indices.end());

    vector<vector<Point2f>> trainCorners, valCorners;
    vector<String> trainPaths, valPaths;
    for (int idx : trainIdx)
    {
        trainCorners.push_back(pointsCorners[idx]);
        trainPaths.push_back(imagesPath[idx]);
    }
    for (int idx : valIdx)
    {
        valCorners.push_back(pointsCorners[idx]);
        valPaths.push_back(imagesPath[idx]);
    }

    cout << "样本分割: 训练集 " << trainCorners.size() << " 张, 验证集 " << valCorners.size()
         << " 张 (随机种子=" << RANDOM_SEED << ")" << endl;

    //[4] 开始标定图像（仅使用训练集）
    vector<vector<Point3f>> pointsObject; // 标定板角点的三维坐标
    vector<Mat> rvecsMat;                 // 训练集图像的旋转向量
    vector<Mat> tvecsMat;                 // 训练集图像的平移向量

    // 构建棋盘格3D点模板（所有图片共用同一组3D点）
    vector<Point3f> tempPointSet;
    for (int i = 0; i < sizeBoard.height; i++)
    {
        for (int j = 0; j < sizeBoard.width; j++)
        {
            Point3f realPoint;
            // 假设标定板放在世界坐标系中z=0的平面上
            realPoint.x = i * sizeSquare.width;
            realPoint.y = j * sizeSquare.height;
            realPoint.z = 0;
            tempPointSet.push_back(realPoint);
        }
    }
    for (size_t t = 0; t < trainCorners.size(); t++)
    {
        pointsObject.push_back(tempPointSet);
    }

    calibrateCamera(pointsObject, trainCorners, sizeImage, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0); // 开始标定

    cout << "相机标定完成!" << endl;
    cout << "正在评价标定结果 | Loading...." << endl;

    ofstream fout("../res/calibration/assessment.txt"); // 保存标定的评价结果

    cout << "========== 标定结果评价 ==========" << endl;
    fout << "========== 标定结果评价 ==========" << endl;
    cout << "相机标定分辨率: " << sizeImage.width << "x" << sizeImage.height << "\n";
    fout << "相机标定分辨率: " << sizeImage.width << "x" << sizeImage.height << "\n";
    cout << "样本分割: 训练集 " << trainCorners.size() << " 张, 验证集 " << valCorners.size()
         << " 张 (随机种子=" << RANDOM_SEED << ")" << endl;
    fout << "样本分割: 训练集 " << trainCorners.size() << " 张, 验证集 " << valCorners.size()
         << " 张 (随机种子=" << RANDOM_SEED << ")" << endl
         << endl;

    // [新增] 用solvePnP求验证集外参（验证集未参与标定，需反推外参才能计算重投影误差）
    vector<vector<Point3f>> valObjectPoints;
    vector<Mat> valRvecs, valTvecs;
    for (size_t i = 0; i < valCorners.size(); i++)
    {
        valObjectPoints.push_back(tempPointSet);
        Mat rvec, tvec;
        solvePnP(tempPointSet, valCorners[i], cameraMatrix, distCoeffs, rvec, tvec);
        valRvecs.push_back(rvec);
        valTvecs.push_back(tvec);
    }

    // 训练集RMSE
    double trainRMSE = computeRMSE(pointsObject, trainCorners, rvecsMat, tvecsMat,
                                   cameraMatrix, distCoeffs, trainPaths, fout, "训练集");

    // 验证集RMSE
    double valRMSE = computeRMSE(valObjectPoints, valCorners, valRvecs, valTvecs,
                                 cameraMatrix, distCoeffs, valPaths, fout, "验证集");

    // 过拟合判断
    fout << "过拟合判断: " << endl;
    cout << "过拟合判断: " << endl;
    double ratio = (trainRMSE > 1e-6) ? (valRMSE / trainRMSE) : 0.0;
    if (valRMSE > trainRMSE * 1.5)
    {
        cout << "  ⚠ 验证集RMSE是训练集的" << ratio << "倍，可能过拟合" << endl;
        fout << "  ⚠ 验证集RMSE是训练集的" << ratio << "倍，可能过拟合" << endl;
        cout << "  建议: 增加标定图片数量，或检查验证集图片质量" << endl;
        fout << "  建议: 增加标定图片数量，或检查验证集图片质量" << endl;
    }
    else
    {
        cout << "  ✓ 验证集RMSE与训练集接近，标定参数泛化能力良好" << endl;
        fout << "  ✓ 验证集RMSE与训练集接近，标定参数泛化能力良好" << endl;
    }
    cout << "  训练集RMSE: " << trainRMSE << " 像素 (" << trainCorners.size() << "张)" << endl;
    fout << "  训练集RMSE: " << trainRMSE << " 像素 (" << trainCorners.size() << "张)" << endl;
    cout << "  验证集RMSE: " << valRMSE << " 像素 (" << valCorners.size() << "张)" << endl;
    fout << "  验证集RMSE: " << valRMSE << " 像素 (" << valCorners.size() << "张)" << endl;
    cout << "  比值: " << ratio << endl;
    fout << "  比值: " << ratio << endl
         << endl;

    // 标定参数输出
    Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
    fout << "========== 标定参数 ==========" << endl;
    fout << "相机内参数矩阵: " << endl;
    fout << cameraMatrix << endl
         << endl;
    fout << "畸变系数: \n";
    fout << distCoeffs << endl
         << endl
         << endl;
    fout << "----- 训练集外参(calibrateCamera) -----" << endl;
    for (size_t i = 0; i < trainCorners.size(); i++)
    {
        string filename = getFilename(trainPaths[i]);
        fout << "图像[" << filename << "]的旋转向量: " << endl;
        fout << rvecsMat[i] << endl;
        /* 将旋转向量转换为相对应的旋转矩阵 */
        Rodrigues(rvecsMat[i], rotation_matrix);
        fout << "图像[" << filename << "]的旋转矩阵: " << endl;
        fout << rotation_matrix << endl;
        fout << "图像[" << filename << "]的平移向量: " << endl;
        fout << tvecsMat[i] << endl
             << endl;
    }
    fout << "----- 验证集外参(solvePnP) -----" << endl;
    for (size_t i = 0; i < valCorners.size(); i++)
    {
        string filename = getFilename(valPaths[i]);
        fout << "图像[" << filename << "]的旋转向量: " << endl;
        fout << valRvecs[i] << endl;
        Rodrigues(valRvecs[i], rotation_matrix);
        fout << "图像[" << filename << "]的旋转矩阵: " << endl;
        fout << rotation_matrix << endl;
        fout << "图像[" << filename << "]的平移向量: " << endl;
        fout << valTvecs[i] << endl
             << endl;
    }

    fout << endl;

    //[5] 将相机标定参数缩放至目标分辨率（icar使用320x240）
    Size sizeTarget = Size(320, 240); // 目标分辨率
    Mat cameraMatrixScaled = cameraMatrix.clone();
    if (sizeImage.width != sizeTarget.width || sizeImage.height != sizeTarget.height)
    {
        double scale_x = sizeTarget.width / (double)sizeImage.width;
        double scale_y = sizeTarget.height / (double)sizeImage.height;
        cameraMatrixScaled.at<double>(0, 0) *= scale_x; // fx
        cameraMatrixScaled.at<double>(0, 2) *= scale_x; // cx
        cameraMatrixScaled.at<double>(1, 1) *= scale_y; // fy
        cameraMatrixScaled.at<double>(1, 2) *= scale_y; // cy
        cout << "内参矩阵已缩放: " << sizeImage.width << "x" << sizeImage.height
             << " -> " << sizeTarget.width << "x" << sizeTarget.height << endl;
        fout << "内参矩阵已缩放: " << sizeImage.width << "x" << sizeImage.height
             << " -> " << sizeTarget.width << "x" << sizeTarget.height << endl;
    }

    // 将缩放后的相机标定参数写入xml文件
    FileStorage fs1("../res/calibration/calibration.xml", FileStorage::WRITE);
    fs1 << "cameraMatrix" << cameraMatrixScaled;
    fs1 << "distCoeffs" << distCoeffs;
    fs1.release();

    //[6] 开始矫正图像
    cout << "正在显示图像矫正结果..." << endl;
    for (int i = 0; i < imagesPath.size(); i++)
    {
        Mat imageSource = imread(imagesPath[i]); // OpenCV读取图像
        Mat mapx = Mat(sizeImage, CV_32FC1);     // 经过矫正后的X坐标重映射参数
        Mat mapy = Mat(sizeImage, CV_32FC1);     // 经过矫正后的Y坐标重映射参数
        Mat R = Mat::eye(3, 3, CV_32F);          // 内参矩阵与畸变矩阵之间的旋转矩阵

        // 采用initUndistortRectifyMap + remap进行图像矫正
        initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, sizeImage, CV_32FC1, mapx, mapy);
        Mat imageCorrect = imageSource.clone();
        remap(imageSource, imageCorrect, mapx, mapy, INTER_LINEAR);

        show.setNewWindow(1, "imgSource", imageSource);   // 添加需要显示的图像
        show.setNewWindow(2, "imgCorrect", imageCorrect); // 添加需要显示的图像
        show.show();                                      // 图像窗口显示

        // 存储矫正图像
        string filename = getFilename(imagesPath[i]);
        string img_path = "../res/calibration/correct/" + filename;
        if (stat(img_path.c_str(), &buffer) != 0) // 判断文件夹是否存在
        {
            string command;
            command = "mkdir -p ../res/calibration/correct/";
            system(command.c_str()); // 利用os创建文件夹
        }
        imwrite(img_path, imageCorrect);

        waitKey(500); // 延时2s
    }

    return true;
}
/**
 * @brief 矫正图像
 *
 * @param imagesPath 图像路径
 */
void imageCorrecte(string imagesPath)
{
    Size sizeImage; // 图像的尺寸
    Show show(2);   // 图像显示窗口重绘
    // 读取xml中的相机标定参数
    Mat matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 摄像机内参矩阵
    Mat coeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); // 相机的畸变矩阵
    FileStorage fs;
    fs.open("../res/calibration/valid/calibration.xml", FileStorage::READ);
    fs["cameraMatrix"] >> matrix;
    fs["distCoeffs"] >> coeffs;

    Mat imageSource = imread(imagesPath); // 读取矫正图像
    sizeImage.width = imageSource.cols;
    sizeImage.height = imageSource.rows;
    Mat mapx = Mat(sizeImage, CV_32FC1); // 经过矫正后的X坐标重映射参数
    Mat mapy = Mat(sizeImage, CV_32FC1); // 经过矫正后的Y坐标重映射参数
    Mat R = Mat::eye(3, 3, CV_32F);      // 内参矩阵与畸变矩阵之间的旋转矩阵

    Mat imageCorrect = imageSource.clone();
    // 采用initUndistortRectifyMap+remap进行图像矫正
    initUndistortRectifyMap(matrix, coeffs, R, matrix, sizeImage, CV_32FC1, mapx, mapy);
    remap(imageSource, imageCorrect, mapx, mapy, INTER_LINEAR);

    // 采用undistort进行图像矫正
    //  undistort(imageSource, imageCorrect, matrix, coeffs);

    show.setNewWindow(0, "imgSource", imageSource);   // 添加需要显示的图像
    show.setNewWindow(1, "imgCorrect", imageCorrect); // 添加需要显示的图像
    show.show();                                      // 图像窗口显示

    while (1)
    {
        waitKey(200);
    }
}

/**
 * @brief Get the Filename object
 *
 * @param path 文件地址
 * @return 文件名称
 */
string getFilename(string sourcePath)
{
    // 获取不带路径的文件名
    string::size_type iPos = sourcePath.find_last_of('/') + 1;
    string filename = sourcePath.substr(iPos, sourcePath.length() - iPos);

    return filename;
}