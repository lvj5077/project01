//
// Created by jin on 4/1/19.
//

#ifndef CMSC591PART1_SLAMBASE_H
#define CMSC591PART1_SLAMBASE_H

# pragma once

// 各种头文件
// C++标准库
#include <fstream>
#include <vector>
#include <map>
using namespace std;

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>


// // PCL
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/common/transforms.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/filters/voxel_grid.h>


#include <iostream>
#include <string>
// 类型定义
// typedef pcl::PointXYZRGBA PointT;
// typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参结构
struct CAMERA_INTRINSIC_PARAMETERS
{
    double cx, cy, fx, fy, scale;
};

// 帧结构
struct FRAME
{
    int frameID;
    cv::Mat rgb, depth; //该帧对应的彩色图与深度图
    cv::Mat desp;       //特征描述子
    vector<cv::KeyPoint> kp; //关键点
};


struct SR4kFRAME
{
    int frameID;
    cv::Mat rgb, depthXYZ; //该帧对应的彩色图与深度图
    cv::Mat desp;       //特征描述子
    vector<cv::KeyPoint> kp; //关键点
};

SR4kFRAME readSRFrame( string inFileName);



void frame2frameT(string firstF, string SecondF, cv::Mat & rvec, cv::Mat & tvec,int showProcess, cv::Mat & rpy );



// 参数读取类
class ParameterReader
{
public:
    ParameterReader( string filename="./parameters.txt" )
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    map<string, string> data;
};





#endif //CMSC591PART1_SLAMBASE_H
