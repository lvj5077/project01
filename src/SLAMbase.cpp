//
// Created by jin on 4/1/19.
//

#include "SLAMbase.h"

#include <algorithm>
#include <iostream>
using namespace std;

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace cv;


// struct SR4kFRAME
// {
//     int frameID;
//     cv::Mat rgb, depthXYZ; //该帧对应的彩色图与深度图
//     cv::Mat desp;       //特征描述子
//     vector<cv::KeyPoint> kp; //关键点
// };


SR4kFRAME readSRFrame( string inFileName){
    SR4kFRAME f;

    int width = 176;
    int height = 144;

    cv::Mat I_gray = cv::Mat::zeros(height,width,CV_64F);

    int size[3] = { width, height, 3 };
    cv::Mat I_depth(3, size, CV_64F, cv::Scalar(10));

    ifstream inFile(inFileName);
    string str;

    int lineIdx = 0;


    while (getline(inFile, str))
    {
        lineIdx ++;
        if (lineIdx > 0 && lineIdx<(height+1))
        {
            for (int i = 0; i < width; i++)
            {
                inFile >> I_depth.at<double>(int(lineIdx-1),i,2) ;
            }
        }

        if (lineIdx > (height+1)*1 && lineIdx<(height+1)*2)
        {
            for (int i = 0; i < width; i++)
            {
                inFile >> I_depth.at<double>(int(lineIdx-1 -(height+1)*1 ),i,0) ;
            }
        }
        if (lineIdx > (height+1)*2 && lineIdx<(height+1)*3)
        {
            for (int i = 0; i < width; i++)
            {
                inFile >> I_depth.at<double>(int(lineIdx-1 -(height+1)*2 ),i,1);
            }
        }

        if (lineIdx > (height+1)*3 && lineIdx<(height+1)*4)
        {
            for (int i = 0; i < width; i++)
            {
                inFile >> I_gray.at<double>(int(lineIdx-1 -(height+1)*3 ),i,0) ;
            }
        }

    }

    // I_gray = I_gray/256;
    // I_gray.convertTo(I_gray,CV_8U);

    I_gray.convertTo(I_gray, CV_8U, 1.0 / 256, 0);

    equalizeHist( I_gray, I_gray );

    f.rgb = I_gray.clone();
    f.depthXYZ = I_depth.clone();

    return f;
}


void frame2frameT(string firstF, string SecondF, cv::Mat & rvec, cv::Mat & tvec,int showProcess, cv::Mat & rpy ){


    // // cout  << "my frame2frameT function"<<endl;

    double depthL = 0.180;
    double depthH = 7.000;


    CAMERA_INTRINSIC_PARAMETERS C;

    C.cx = 88.5320;
    C.cy = 72.5102;
    C.fx = 222.6132;
    C.fy = 225.6439;

    double camera_matrix_data[3][3] = {
        {C.fx, 0, C.cx},
        {0, C.fy, C.cy},
        {0, 0, 1}
    };
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );



    // 第一个帧的三维点
    vector<cv::Point3f> firstPts;
    // 第二个帧的三维点
    vector<cv::Point3f> secondPts;
    // 第二个帧的图像点
    vector< cv::Point2f> pts_img;



    // string inFileName;

    cout  << "load in " << firstF <<endl;
    cout  << "load in " <<  SecondF <<endl<<endl;
    SR4kFRAME f1 = readSRFrame(firstF) ;
    SR4kFRAME f2 = readSRFrame(SecondF) ;


    cv::Mat rgb1 = f1.rgb;
    cv::Mat rgb2 = f2.rgb;

    cv::Mat depth1 = f1.depthXYZ;
    cv::Mat depth2 = f2.depthXYZ;


    cv::Ptr<cv::FeatureDetector> _detector;
    cv::Ptr<cv::DescriptorExtractor> _descriptor;
    cv::initModule_nonfree();
    _detector = cv::FeatureDetector::create( "SIFT" );
    // _detector->set("contrastThreshold",0.02);
    // _detector->set("edgeThreshold",7);


    _descriptor = cv::DescriptorExtractor::create( "SIFT" );
    vector< cv::KeyPoint > kp1, kp2; 
    _detector->detect( rgb1, kp1 );
    _detector->detect( rgb2, kp2 );


    cv::Mat imgShow;
    if(showProcess>0){
        cout <<"Key points of two images: "<<kp1.size()<<", "<<kp2.size()<<endl;
        cv::drawKeypoints( rgb1, kp1, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        cv::imshow( "keypoints", imgShow );
        cv::waitKey(0); //暂停等待一个按键  
    }


   
    // 计算描述子
    cv::Mat desp1, desp2;
    _descriptor->compute( rgb1, kp1, desp1 );
    _descriptor->compute( rgb2, kp2, desp2 );

    vector< cv::DMatch > matches; 
    cv::FlannBasedMatcher matcher;
    matcher.match( desp1, desp2, matches );
    cv::Mat imgMatches;
    if(showProcess>0){
        cout <<"Find total "<<matches.size()<<" matches."<<endl;
        cv::drawMatches( rgb1, kp1, rgb2, kp2, matches, imgMatches );
        cv::imshow( "matches", imgMatches );
        cv::waitKey( 0 );
    }


    vector< cv::DMatch > goodMatches;
    double minDis = 9999;
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }

    minDis = max(1.0,minDis);
    for ( size_t i=0; i<matches.size(); i++ )
    {
        cv::Point2f p1 = kp1[matches[i].queryIdx].pt;
        cv::Point2f p2 = kp2[matches[i].trainIdx].pt;

        double d1 = depth1.at<double>(int(p1.x),int(p1.y),2);
        double d2 = depth2.at<double>(int(p2.x),int(p2.y),2);

        double pixeldist = (p1.x - p2.x)*(p1.x - p2.x)+(p1.y - p2.y)*(p1.y - p2.y);

        if (matches[i].distance < 5*minDis && d1>depthL&&d1<depthH && d2>depthL&&d2<depthH){
            goodMatches.push_back( matches[i] );


            cv::Point3f pd1;
            pd1.x = depth1.at<double>(int(p1.x),int(p1.y),0);
            pd1.y = depth1.at<double>(int(p1.x),int(p1.y),1);
            pd1.z = depth1.at<double>(int(p1.x),int(p1.y),2);
            firstPts.push_back( pd1 );

            pts_img.push_back( p2 );

            cv::Point3f pd2;
            pd2.x = depth2.at<double>(int(p2.x),int(p2.y),0);
            pd2.y = depth2.at<double>(int(p2.x),int(p2.y),1);
            pd2.z = depth2.at<double>(int(p2.x),int(p2.y),2);
            secondPts.push_back( pd2 );

            // // // cout  << pd2 <<endl;
        }
    }


    if(showProcess>0){
        cout <<"good matches="<<goodMatches.size()<<endl;
        cv::drawMatches( rgb1, kp1, rgb2, kp2, goodMatches, imgMatches );
        cv::imshow( "good matches", imgMatches );
        cv::waitKey(0);
        cout  << "first visual stage done!"<<endl<<endl;
    }


    cout <<"good matches="<<goodMatches.size()<<endl;
/*==============================================================================*/

    // cv::Mat rvec, tvec, inliers;
    cv::Mat inliers;
    cv::solvePnPRansac( firstPts, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 300, 10, 100, inliers );

    // cout <<"inliers: "<<inliers.rows<<endl;
    // cout <<"R="<<rvec<<endl;
    // cout <<"t="<<tvec<<endl;

    cv::Mat R;
    cv::Rodrigues(rvec, R); // R is 3x3

    float roll,pitch,yaw;
    float sy;

    cv::Mat T = cv::Mat::eye(4,4,CV_64F);

    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(T(cv::Rect(3, 0, 1, 3)));
    

    double sumDist = 0;
    double sumError = 0;
    int countN = 0;
    int goodinlierN = 0;
    double goodsumError = 0;
    vector< cv::DMatch > matchesShow;
    double pxm = 0;
    double pym = 0;
    double pzm = 0;
    double sumDistG = 0;
    double sumErrorG = 0;
    vector<cv::Point3f> firstPtsG;
    vector<cv::Point3f> secondPtsG;
    vector< cv::Point2f> pts_imgG;

    for (size_t i=0; i<inliers.rows; i++)
    {
        // matchesShow.push_back( goodMatches[inliers.ptr<int>(i)[0]] );    
        // query 是第一个, train 是第二个
        cv::Point2f p1 = kp1[goodMatches[ inliers.ptr<int>(i)[0] ].queryIdx].pt;
        cv::Point2f p2 = kp2[goodMatches[ inliers.ptr<int>(i)[0] ].trainIdx].pt;

        cv::Point3f pd1;
        pd1.x = depth1.at<double>(int(p1.x),int(p1.y),0);
        pd1.y = depth1.at<double>(int(p1.x),int(p1.y),1);
        pd1.z = depth1.at<double>(int(p1.x),int(p1.y),2);

        cv::Point3f pd2;
        pd2.x = depth2.at<double>(int(p2.x),int(p2.y),0);
        pd2.y = depth2.at<double>(int(p2.x),int(p2.y),1);
        pd2.z = depth2.at<double>(int(p2.x),int(p2.y),2);

        cv::Mat ptMat = (cv::Mat_<double>(4, 1) << pd1.x, pd1.y, pd1.z, 1);
        cv::Mat dstMat = T*ptMat;
        cv::Point3f projPd1(dstMat.at<double>(0,0), dstMat.at<double>(1,0),dstMat.at<double>(2,0));
        // // // cout  << "projPd1 "<<projPd1 <<endl;
        if(showProcess>0){

            cout  << "p1" << p1 << endl;
            cout  << "p2" << p2 << endl;
            cout  << "pd1" << pd1 << endl;
            cout  << "pd2" << pd2 << endl;
            cout  << "(pd1-pd2) "<< norm(pd1-pd2)*1000 <<"mm" <<endl;
            cout  << "(pd1*T-pd2) "<< norm(projPd1-pd2)*1000 <<"mm"<<endl<<endl;
        }



        
        // sumError = sumError + norm(projPd1-pd2)*1000;
        countN++;
        if ( norm(pd1-pd2)*1000 < 500 && norm(p1-p2)<60){
            goodinlierN++;
            goodsumError = goodsumError + norm(projPd1-pd2)*1000;
            sumDist = sumDist + norm(pd1-pd2)*1000;
            firstPtsG.push_back( pd1 );
            secondPtsG.push_back( pd2 );
            pts_imgG.push_back( p2 );
            matchesShow.push_back( goodMatches[inliers.ptr<int>(i)[0]] );    
        }
    }
    cout  << "goodinlierN "<< goodinlierN <<endl;
    // cout  << "goodsumError "<< goodsumError/goodinlierN <<endl;
    cout  << "totalinlierN "<< countN <<endl;

    // showProcess = 1;
    if(1){
        cv::drawMatches( rgb1, kp1, rgb2, kp2, matchesShow, imgMatches );
        cv::imshow( "3d-2d inlier matches", imgMatches );
        cv::waitKey( 0 );
    }
    cout << endl<<"compare~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;

    cout  << "solvePnPRansac inlier  avg error "<< goodsumError/goodinlierN <<"mm" <<endl;
    cout  << "solvePnPRansac inlier  avg dist "<< sumDist/goodinlierN  <<"mm" <<endl;
    cout  << "solvePnPRansac inlier  roll "<< roll <<" ; pitch " <<pitch<<" ; yaw "<<yaw<<endl<<endl;
    // cout  << "3D-2D solver done!"<<endl<<endl;


 
    rpy.at<double>(0,0) = roll;
    rpy.at<double>(1,0) = pitch;
    rpy.at<double>(2,0) = yaw;


    if (goodinlierN>5){

        cv::solvePnPRansac( firstPtsG, pts_imgG, cameraMatrix, cv::Mat(), rvec, tvec, false, 300, 10, 100, inliers );

        cv::Rodrigues(rvec, R); // R is 3x3
        sy= sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
        roll = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        pitch = atan2(-R.at<double>(2,0), sy);
        yaw = atan2(R.at<double>(1,0), R.at<double>(0,0));

        cout  << "Good solvePnPRansac inlier  roll "<< roll <<" ; pitch " <<pitch<<" ; yaw "<<endl<<yaw<<endl;


        //3d-3d=============================================================================
        cv::Mat outM3by4G;// = cv::Mat::zeros(3,4,CV_64F);
        cv::Mat inliers3dG;
        cv::estimateAffine3D(secondPtsG,firstPtsG, outM3by4G, inliers3dG, 3, 0.999); 
        R= outM3by4G(cv::Rect(0,0,3,3));
        sy= sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
        roll = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        pitch = atan2(-R.at<double>(2,0), sy);
        yaw = atan2(R.at<double>(1,0), R.at<double>(0,0));

        cout  << "Good estimateAffine3D inlier  roll "<< roll <<" ; pitch " <<pitch<<" ; yaw "<<yaw<<endl<<endl;

        T = cv::Mat::eye(4,4,CV_64F);
        cv::Mat rmat = outM3by4G(cv::Rect(0,0,3,3));
        cv::Mat tvecN = outM3by4G(cv::Rect(3,0,1,3));
        rmat.copyTo(T(cv::Rect(0, 0, 3, 3)));
        tvecN.copyTo(T(cv::Rect(3, 0, 1, 3)));
        /*reprojection error*/
        sumDistG = 0;
        sumErrorG = 0;
        for (int i=0;i<goodinlierN;i++){
            cv::Point3f pd1 = firstPtsG[i];
            cv::Point3f pd2 = secondPtsG[i];

            cv::Mat ptMat = (cv::Mat_<double>(4, 1) << pd1.x, pd1.y, pd1.z, 1);
            cv::Mat dstMat = T*ptMat;
            cv::Point3f projPd1(dstMat.at<double>(0,0), dstMat.at<double>(1,0),dstMat.at<double>(2,0));
            // // // cout  << "projPd1 "<<projPd1 <<endl;
            if(showProcess>0){
                cout  << "pd1" << pd1 << endl;
                cout  << "pd2" << pd2 << endl;
                cout  << "(pd1-pd2) "<< norm(pd1-pd2)*1000 <<"mm" <<endl;
                cout  << "(pd1*T-pd2) "<< norm(projPd1-pd2)*1000 <<"mm"<<endl<<endl;
            }



            sumDistG = sumDistG + norm(pd1-pd2)*1000;
            sumErrorG = sumErrorG + norm(projPd1-pd2)*1000;
        }
        cout  << "Good estimateAffine3D inlier  avg error "<< sumErrorG/goodinlierN <<"mm" <<endl;
        cout  << "Good estimateAffine3D inlier  avg dist "<< sumDistG/goodinlierN  <<"mm" <<endl;
        cout  << "Good estimateAffine3D inlier  roll "<< roll <<" ; pitch " <<pitch<<" ; yaw "<<yaw<<endl<<endl;


        //SVD=============================================================================
        cv::Mat firstM = cv::Mat::zeros(goodinlierN,4,CV_64F);
        cv::Mat secondM = cv::Mat::zeros(goodinlierN,4,CV_64F);
        for (int i=0;i<goodinlierN;i++){
            firstM.at<double>(i,3) = 1.0;
            secondM.at<double>(i,3) = 1.0;

            firstM.at<double>(i,0) = (firstPtsG[i]).x;
            secondM.at<double>(i,0) = (secondPtsG[i]).x;

            firstM.at<double>(i,1) = (firstPtsG[i]).y;
            secondM.at<double>(i,1) = (secondPtsG[i]).y;

            firstM.at<double>(i,2) = (firstPtsG[i]).z;
            secondM.at<double>(i,2) = (secondPtsG[i]).z;
        }
        cv::Mat Tm;
        // cout <<"first: "<<endl<<firstM<<endl;
        // cout <<"second: "<<endl<<secondM<<endl;
        cv::solve(firstM,secondM,Tm,DECOMP_SVD);
        // cv::transpose(Tm,Tm);
        // cout << Tm<<endl;
        cv::transpose(Tm,Tm);
        // cout << Tm<<endl;
        R= Tm(cv::Rect(0,0,3,3));
        sy= sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
        roll = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        pitch = atan2(-R.at<double>(2,0), sy);
        yaw = atan2(R.at<double>(1,0), R.at<double>(0,0));

        

        rpy.at<double>(0,0) = roll;
        rpy.at<double>(1,0) = pitch;
        rpy.at<double>(2,0) = yaw;
        tvecN = Tm(cv::Rect(3,0,1,3));
        /*reprojection error*/
        sumDistG = 0;
        sumErrorG = 0;
        for (int i=0;i<goodinlierN;i++){
            cv::Point3f pd1 = firstPtsG[i];

            cv::Point3f pd2 = secondPtsG[i];

            cv::Mat ptMat = (cv::Mat_<double>(4, 1) << pd1.x, pd1.y, pd1.z, 1);
            cv::Mat dstMat = Tm*ptMat;
            cv::Point3f projPd1(dstMat.at<double>(0,0), dstMat.at<double>(1,0),dstMat.at<double>(2,0));
            // // // cout  << "projPd1 "<<projPd1 <<endl;
            if(showProcess>0){
                cout  << "pd1" << pd1 << endl;
                cout  << "pd2" << pd2 << endl;
                cout  << "(pd1-pd2) "<< norm(pd1-pd2)*1000 <<"mm" <<endl;
                cout  << "(pd1*T-pd2) "<< norm(projPd1-pd2)*1000 <<"mm"<<endl<<endl;
            }



            sumDistG = sumDistG + norm(pd1-pd2)*1000;
            sumErrorG = sumErrorG + norm(projPd1-pd2)*1000;
        }
        cout  << "Good SVD inlier  avg error "<< sumErrorG/goodinlierN <<"mm" <<endl;
        cout  << "Good SVD inlier  avg dist "<< sumDistG/goodinlierN  <<"mm" <<endl;
        cout  << "Good SVD inlier  roll "<< roll <<" ; pitch " <<pitch<<" ; yaw "<<yaw<<endl<<endl;
    }




    cout << "output"<<endl;
    cout << "roll "<< rpy.at<double>(0,0) <<" ; pitch " <<rpy.at<double>(1,0)<<" ; yaw "<<rpy.at<double>(1,0)<<endl;
    cout << endl<<endl;

}