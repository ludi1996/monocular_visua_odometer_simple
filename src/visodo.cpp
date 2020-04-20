#include "vo_features.h"

using namespace cv;
using namespace std;

#define MAX_FRAME 1000 // 处理1000张图片
#define MIN_NUM_FEAT 2000  // 每次计算中 图片中特征点的数量不得低于2000

int main( int argc, char** argv )	{

  Mat img_1, img_2; // 根据最初两帧进行初始化
  Mat R_f, t_f; // 最终的位姿 R t

  double scale = 1.00; // 单目相机两帧之间的t是没有尺度的 要得到真实的t 需要乘以一个尺度
  char filename1[200];
  char filename2[200]; //前后两张图片的文件名
  sprintf(filename1, "../database/img/%06d.png", 0);
  sprintf(filename2, "../database/img/%06d.png", 1); //指定文件路径

  //这里是结果可视化中用到的变量和参数
  char text[100];
  int fontFace = FONT_HERSHEY_PLAIN;
  double fontScale = 1;
  int thickness = 1;  
  cv::Point textOrg(10, 50);

  //从数据集中读前两张图片
  Mat img_1_c = imread(filename1);
  Mat img_2_c = imread(filename2);

  if ( !img_1_c.data || !img_2_c.data ) { 
    std::cout<< " --(!) Error reading images " << std::endl; return -1; //没读到图片报错
  }

  //图片处理过程都用的是灰度图像
  cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
  cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);

  // 特征提取，并且用光流法进行特征匹配
  vector<Point2f> points1, points2;        //用 vector<Point2f> 储存特征点的坐标
  featureDetection(img_1, points1);        //对img_1进行特征提取 方法在vo_features.h
  vector<uchar> status;     // vector<uchar> 型数据 用于光流法函数
  featureTracking(img_1,img_2,points1,points2, status); //用光流法在img_2中找到img_1提取的那些特征点 方法在vo_features.h
  // 这时得到两组特征点 points1, points2
  // points1, points2 未匹配的特征点都被剔除掉了 只有匹配好的点剩下
  // points1, points2 的size是一样的 而且是一一对应的

  // 这里把calib.txt文件中的相机内外参手动定义出来
  double focal = 718.8560;      // 焦距
  cv::Point2d pp(607.1928, 185.2157);       // 径向畸变 这里的数据格式按照opencv的要求

  // 计算Essential矩阵 从E中恢复出R和t
  Mat E, R, t, mask;     // 不用mask 定义出 mask 为空即可
  E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);  // opencv中的函数 有了两帧间匹配的特征点 就能计算Essential了
  recoverPose(E, points2, points1, R, t, focal, pp, mask);      // opencv中的函数 从Essential恢复出 R t

  // 前两帧的初始化完成了 得到初始位姿 准备开始计算后面的所有图片
  Mat prevImage = img_2;
  Mat currImage;                   // 定义previous和current两个图片
  vector<Point2f> prevFeatures = points2;
  vector<Point2f> currFeatures;            // 定义previous和current两个图片中的特征点

  char filename[100];            // 下一张图片的文件名

  R_f = R.clone();
  t_f = t.clone();            // 将初始化的 R t 赋值

  clock_t begin = clock();            // 计时开始

  namedWindow( "Road facing camera", WINDOW_AUTOSIZE );// Create a window for display.
  namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.
  Mat traj = Mat::zeros(600, 600, CV_8UC3);      // display背景 不用管

  // 开始视觉里程计
  for(int numFrame=2; numFrame < MAX_FRAME; numFrame++)	{
  	sprintf(filename, "../database/img/%06d.png", numFrame);
  	Mat currImage_c = imread(filename);             // 读取 current img
  	cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);            // 转成灰度图像
  	vector<uchar> status;            // 定义status 用来特征匹配
  	// 光流法进行特征匹配
  	featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
  	// 第一次迭代时 prevImage是刚才的第二张图片  currImage是这里读取的第三张图片 图片序号从0开始
  	// prevFeatures就是刚才的points2  currFeatures是计算得到输出的 在第三张图中 与points2相匹配的一组特征点
  	// 同样这里已经剔除了未匹配的点 这时的特征点数不能低于设定的阈值2000 如果低于了2000 就要重新进行特征检测
  	// a redetection is triggered in case the number of feautres being trakced go below a particular threshold
  	if (prevFeatures.size() < MIN_NUM_FEAT)	{
      featureDetection(prevImage, prevFeatures);
      featureTracking(prevImage,currImage,prevFeatures,currFeatures, status);
  	}
  	// 一样的 恢复R t
  	E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
  	recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
  	// 这里要通过数据集中给的真实的里程计数据 计算出scale 当前两帧间的scale
  	scale = getAbsoluteScale(numFrame, 0, t.at<double>(2));

  	// 有了两帧间的R t 和 scale 可以更新最终的位姿  R_f t_f
  	// scale不能太小 scale过小说明两帧间没有运动 计算得到的 R t 不准确
    if ((scale>0.2)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
      t_f = t_f + scale*(R_f*t);
      R_f = R*R_f;
    }
    else {
      //cout << "scale below 0.2, or incorrect translation" << endl;
    }



    // current pic的图像和特征点赋值给 previous pic 以进行下一次迭代
    prevImage = currImage.clone();
    prevFeatures = currFeatures;



    // 下面是可视化部分 显示当前帧图片 并绘制当前帧位置 得到轨迹
    int x = int(t_f.at<double>(0)) + 300;
    int y = -1*int(t_f.at<double>(2)) + 500;
    circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);

    rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
    sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
    putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

    imshow( "Road facing camera", currImage_c );
    imshow( "Trajectory", traj );

    waitKey(1);

  }

  // 计时结束 显示总共的计算时间
  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Total time taken: " << elapsed_secs << "s" << endl;

  //cout << R_f << endl;
  //cout << t_f << endl;

  return 0;
}