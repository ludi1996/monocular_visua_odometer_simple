

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"


#include <iostream>
#include <ctype.h>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

// 特征提取函数 输入img_1 输出特征点points1
void featureDetection(Mat img_1, vector<Point2f>& points1)	{   //uses FAST as of now, modify parameters as necessary
    vector<KeyPoint> keypoints_1;       // vector<KeyPoint> 用来储存特征点
    int fast_threshold = 20;            // 使用fast特征 设置阈值为20
    bool nonmaxSuppression = true;      // 采用极大值抑制
    FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
    /*  参数：
        ·image – 输入灰度图像。
        ·keypoints – 检测到的特征点。
        ·threshold – 中心像素的像素值和该像素周围的像素值之差的阈值。
        ·nonmaxSuppression – 是否对特征点采用极大值抑制。
    */
    KeyPoint::convert(keypoints_1, points1, vector<int>());  //转换格式 vector<KeyPoint> to vector<Point2f>
}

// LK光流法 找到下一帧图像中的特征点 并且剔除未匹配的点
// 输入两帧图像和第一帧图像中的特征点 输出第二帧图像中的特征点 并剔除两组点中未匹配的点
void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)	{
//this function automatically gets rid of points for which tracking fails

  vector<float> err;		// 参数
  Size winSize=Size(21,21);		// LK光流法 窗口大小
  // TermCriteria类  作为calcOpticalFlowPyrLK的参数
  // 这是opencv一个模板类 表示迭代结束条件 这里的意思是最多迭代30次 或者 误差小于阈值0.01时停止迭代
  TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
  // LK光流法
  calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);
    /*
    - prevImg
    在时间 t的第一帧
    - nextImg
    在时间 t + dt 的第二帧
    - prevPts
    发现的第一帧的光流点集
    - nextPts
    第二帧的光流点集
    - - status
    状态数组，如果对应特征的光流被发现，数组中的每一个元素都被设置为 1， 否则设置为 0。
    - err
    双精度数组，包含原始图像碎片与移动点之间的差。为可选参数，可以是 NULL .criteria准则，指定在每个金字塔层，为某点寻找光流的迭代过程的终止条件。
    - win_size
    每个金字塔层的搜索窗口尺寸
    - maxLevel
    - 最大的金字塔层数。如果为 0 , 不使用金字塔 (即金字塔为单层), 如果为 1 , 使用两层，下面依次类推。
    */


  // 剔除未匹配的点 剔除跑到图像外的点
  // 遍历每个特征点 检查该特征点是否成功匹配 如果没有就从vector中剔除
  int indexCorrection = 0;
  for( int i=0; i<status.size(); i++)
     {  Point2f pt = points2.at(i - indexCorrection);
     	if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
     		  if((pt.x<0)||(pt.y<0))	{
     		  	status.at(i) = 0;
     		  }
     		  points1.erase (points1.begin() + (i - indexCorrection));
     		  points2.erase (points2.begin() + (i - indexCorrection));
     		  indexCorrection++;
     	}

     }

}

// 获取两帧之间的尺度 数据库中可以下载到真实的里程计数据
// 打开里程计数据文件 读取当前帧和上一帧的空间位置xyz 计算x^2+y^2+z^2 就可得到真实位移的模 即scale
// 有了scale 乘以 归一化的t 就是真实的t
double getAbsoluteScale(int frame_id, int sequence_id, double z_cal)	{

    string line;
    int i = 0;
    ifstream myfile ("../database/00.txt");
    double x =0, y=0, z = 0;
    double x_prev, y_prev, z_prev;
    if (myfile.is_open())
    {
        while (( getline (myfile,line) ) && (i<=frame_id))
        {
            z_prev = z;
            x_prev = x;
            y_prev = y;
            std::istringstream in(line);
            //cout << line << '\n';
            for (int j=0; j<12; j++)  {
                in >> z ;
                if (j==7) y=z;
                if (j==3)  x=z;
            }

            i++;
        }
        myfile.close();
    }

    else {
        cout << "Unable to open file";
        return 0;
    }

    return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;

}
