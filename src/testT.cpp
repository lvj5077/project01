//
// Created by jin on 4/1/110.
//

#include "SLAMbase.h"

#include <algorithm>
#include <iostream>
using namespace std;

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace cv;

#include <fstream>
#include <sstream>

int main( int argc, char** argv )
{

    // /home/jin/Data/RV_Data/Pitch/d1_-40/d1_0001.dat
    // /home/jin/Data/RV_Data/Pitch/d2_-37/d2_0001.dat
    // /home/jin/Data/RV_Data/Pitch/d10_-34/d10_0001.dat
    // /home/jin/Data/RV_Data/Pitch/d4_-31/d4_0001.dat



    // /home/jin/Data/RV_Data/Yaw/d1_44/d1_0001.dat
    // /home/jin/Data/RV_Data/Yaw/d2_41/d2_0001.dat
    // /home/jin/Data/RV_Data/Yaw/d10_108/d10_0001.dat
    // /home/jin/Data/RV_Data/Yaw/d4_1010/d4_0001.dat


    // /home/jin/Data/RV_Data/Translation/Y1/frm_0001.dat
    // /home/jin/Data/RV_Data/Translation/Y2/frm_0001.dat
    // /home/jin/Data/RV_Data/Translation/Y10/frm_0001.dat
    // /home/jin/Data/RV_Data/Translation/Y4/frm_0001.dat

    string firstF = "/Users/lingqiujin/Data/RV_Data/Pitch/d1_-40/d1_0044.dat";
    string secondF = "/Users/lingqiujin/Data/RV_Data/Pitch/d2_-37/d2_0044.dat";
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat rpy=cv::Mat::zeros(3,1,CV_64F);;
    cout << "test"<<endl;
    frame2frameT(firstF, secondF, rvec, tvec,0,rpy);
    cout<<"t="<<tvec<<endl<<endl;
    cout << "roll "<< rpy.at<double>(0,0) <<" ; pitch " <<rpy.at<double>(1,0)<<" ; yaw "<<rpy.at<double>(2,0)<<endl;


    cout << "My test main: "<<endl;

    cv::Mat mroll(cv::Size(10,10),CV_64FC1);
    cv::Mat mpitch(cv::Size(10,10),CV_64FC1);
    cv::Mat myaw(cv::Size(10,10),CV_64FC1);
    cv::Mat mx(cv::Size(10,10),CV_64FC1);
    cv::Mat my(cv::Size(10,10),CV_64FC1);
    cv::Mat mz(cv::Size(10,10),CV_64FC1);


    ofstream myfile;
    myfile.open ("/Users/lingqiujin/Desktop/data.txt");
    

    int gN1 = 0;
    int gN2 = 0;
    for (int g1=0;g1<10;g1++){
        for(int g2=0;g2<10;g2++){
            gN1 = g1+30;
            gN2 = g2+30;
            if (gN1<9){
                firstF = "/Users/lingqiujin/Data/RV_Data/Yaw/d1_44/d1_000"+to_string(gN1)+".dat";
            }else{
                firstF = "/Users/lingqiujin/Data/RV_Data/Yaw/d1_44/d1_00"+to_string(gN1)+".dat";
            }
            
            if (gN2<9){
                secondF = "/Users/lingqiujin/Data/RV_Data/Yaw/d2_41/d2_000"+to_string(gN2)+".dat";
            }else{
                secondF = "/Users/lingqiujin/Data/RV_Data/Yaw/d2_41/d2_00"+to_string(gN2)+".dat";
            }

            frame2frameT(firstF, secondF, rvec, tvec,0,rpy);

            mroll.at<double>(g1,g2) = rpy.at<double>(0,0) ;
            mpitch.at<double>(g1,g2) = rpy.at<double>(1,0) ;
            myaw.at<double>(g1,g2) = rpy.at<double>(2,0) ;

            mx.at<double>(g1,g2) = tvec.at<double>(0,0) ;
            my.at<double>(g1,g2) = tvec.at<double>(1,0) ;
            mz.at<double>(g1,g2) = tvec.at<double>(2,0) ;
            
            stringstream ss;
            ss << rpy.at<double>(0,0)<<","<<rpy.at<double>(1,0)<<","<<rpy.at<double>(2,0)<<","<<tvec.at<double>(0,0) << ","<< tvec.at<double>(1,0)<< "," << tvec.at<double>(2,0);

            string rpy = ss.str();
            myfile << rpy <<"\n";
        }
    }

    myfile.close();


    cv::Scalar mean,stddev;

    cv::meanStdDev(mroll,mean,stddev);
    cout <<"roll  mean: "<<mean.val[0]<<"  std: "<<stddev.val[0]<<endl;
    cv::meanStdDev(mpitch,mean,stddev);
    cout <<"pitch mean: "<<mean.val[0]<<"  std: "<<stddev.val[0]<<endl;
    cv::meanStdDev(myaw,mean,stddev);
    cout <<"Yaw   mean: "<<mean.val[0]<<"  std: "<<stddev.val[0]<<endl;

    // cv::meanStdDev(mx,mean,stddev);
    // cout <<"mx  mean: "<<mean.val[0]<<"  std: "<<stddev.val[0]<<endl;
    // cv::meanStdDev(my,mean,stddev);
    // cout <<"my mean: "<<mean.val[0]<<"  std: "<<stddev.val[0]<<endl;
    // cv::meanStdDev(mz,mean,stddev);
    // cout <<"mz   mean: "<<mean.val[0]<<"  std: "<<stddev.val[0]<<endl;



    return 0;
}
