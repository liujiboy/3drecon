//
//  Image.cpp
//  Document
//
//  Created by  刘骥 on 16/4/28.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#include "Image.hpp"
Image::Image(const std::string&fileName,int i,const cv::Mat_<double>& k,const cv::Mat_<double>& rt)
{
    id=i;
    data=cv::imread(fileName);
    int row=data.rows;
    int col=data.cols;
    kmat=k;
    rtmat=rt;
    pmat=k*rt;
    rmat=rt(cv::Rect(0,0,3,3));
    tmat=rt(cv::Rect(3,0,1,3));
    cv::Mat_<double> c=-rmat.t()*tmat;
    cv::Matx<double, 4, 1> hc(c(0,0),c(1,0),c(2,0),1);
    cameraCenter=cv::Mat(hc);
    gridWidth=floor((col/2.0)+0.5);
    gridHeight=floor((row/2.0)+0.5);
    qf.resize(gridHeight,std::vector<std::set<int>>(gridWidth));
    qt.resize(gridHeight,std::vector<std::set<int>>(gridWidth));
    searched.resize(gridHeight,std::vector<bool>(gridWidth,false));
    //cellLock.resize(row/2,std::vector<std::shared_ptr<std::mutex>>(col/2,std::shared_ptr<std::mutex>(new std::mutex)));
    depth.resize(gridHeight,std::vector<int>(gridWidth,-1));
    
    xaxis(0,0)=rt(0,0);
    xaxis(1,0)=rt(0,1);
    xaxis(2,0)=rt(0,2);
    xaxis(3,0)=0;
    yaxis(0,0)=rt(1,0);
    yaxis(1,0)=rt(1,1);
    yaxis(2,0)=rt(1,2);
    yaxis(3,0)=0;
    zaxis(0,0)=rt(2,0);
    zaxis(1,0)=rt(2,1);
    zaxis(2,0)=rt(2,2);
    zaxis(3,0)=0;

}
void Image::detectFeatures()
{
    //SIFT sift( 0, 3,0.08, 5, 1.6);
//    SIFT sift;
//    vector<KeyPoint> keypoints;
//    sift(data,Mat(),keypoints,descriptors);
//    for (int i=0; i<keypoints.size(); i++) {
//        Feature f;
//        f.image=this;
//        f.x=keypoints[i].pt.x;
//        f.y=keypoints[i].pt.y;
//        features.push_back(f);
//    }
    std::vector<cv::Point2d> corners;
    cv::Mat dst;
    cv::cvtColor(data, dst, cv::COLOR_BGR2GRAY);
    cv::goodFeaturesToTrack(dst, corners, 400, 0.04, 10);
    //std::cerr<<"图像"<<id<<"检测到特征点"<<corners.size()<<std::endl;
    for (int i=0; i<corners.size(); i++) {
                Feature f;
                f.image=this;
                f.x=corners[i].x;
                f.y=corners[i].y;
                features.push_back(f);
    }
    
}
//bool Image::isInEmptyCell(const Feature &f)const
//{
//    //if(!qt[f.y][f.x].empty())
//    //    cout<<"ok"<<endl;
//    return qt[f.y][f.x].empty()&&qf[f.y][f.x].empty();
//}
//void Image::computeEpipolarLine(const Mat3x4 &p2, const Feature &f, Mat3x1 &line)const
//{
//    Mat3x4 p1=pmat;
//    Mat4x1 c1=cameraCenter;
//    Mat3x3 pinvP1=p1.t()*(p1*p1.t()).inv();
//    Mat3x1 x;
//    x(0,0)=f.x;
//    x(1,0)=f.y;
//    x(2,0)=1;
//    line=(p2*c1).cross(p2*pinvP1*x);
//}
//void Image::findFeatures(const Feature &f1,vector<Feature>&featuresNearEpipolarLine)const
//{
//    
//    for (Image *nimage:nimages) {
//        Mat3x1 line;
//        computeEpipolarLine(nimage->pmat,f1, line);
//        double a=line(0,0);
//        double b=line(1,0);
//        double c=line(2,0);
//        for (const Feature &f:nimage->features) {
//            double d=abs(a*f.x+b*f.y+c)/sqrt(a*a+b*b);
//            if(d<=1)
//            {
//                featuresNearEpipolarLine.push_back(f);
//            }
//        }
//
//    }
//}
double Image::getDistanceToCameraCenter(const Mat4x1 &point)const
{
    return norm(cameraCenter-point);
}
Mat3x1 Image::project(const Mat4x1 &point)const
{
    Mat3x1 m=pmat*point;
    //cout<<m(2,0)<<endl;
    m=m/m(2,0);
    //m=m/10;
    //cout<<m<<endl;
    return m;
}
void Image::grabTex(const Mat4x1 coord,const Mat4x1&pxaxis,const Mat4x1&pyaxis,Tex&pTex)const
{
    //pTex.points.clear();
    //pTex.values.clear();
    Mat3x1 center=project(coord);
    Mat3x1 dx=project(coord+pxaxis)-center;
    Mat3x1 dy=project(coord+pyaxis)-center;
    int k=0;
    for(int i=-3;i<=3;i++)
        for(int j=-3;j<=3;j++)
        {
            Mat3x1 p=center+j*dx+i*dy;
            cv::Mat v1,v2;
            double px=p(0,0);
            double py=p(1,0);
            if (px<0||px>data.cols-1||py<0||py>data.rows-1||std::isnan(px)||std::isnan(py)) {
                //pTex.values.push_back(-1);
                //pTex.values.push_back(-1);
                //pTex.values.push_back(-1);
                pTex.values[k++]=-1;
                pTex.values[k++]=-1;
                pTex.values[k++]=-1;
            }else{
                double r,g,b;
                getColor(px,py,r,g,b);
                pTex.values[k++]=r;
                pTex.values[k++]=g;
                pTex.values[k++]=b;
//                Mat v;
//                cv::getRectSubPix(data, Size(1,1), Point2f(px,py), v);
//                pTex.values[k++]=v.at<Vec3b>(0,0)[0];
//                pTex.values[k++]=v.at<Vec3b>(0,0)[1];
//                pTex.values[k++]=v.at<Vec3b>(0,0)[2];
            }
        }
}
void Image::getColor(double x,double y,double&r,double&g,double&b)const 
{
    cv::Vec3b v;
    int x1=(int)x,y1=(int)y,x2=(int)(x+0.5),y2=(int)(y+0.5);
    if(x1==x2&&y1==y2)
    {
        v=data.at<cv::Vec3b>(y, x);
    }else if(x1==x2)
    {
        cv::Vec3b q11=data.at<cv::Vec3b>(y1, x1);
        cv::Vec3b q12=data.at<cv::Vec3b>(y2, x1);
        v=(y2-y)/(y2-y1)*q11+(y-y1)/(y2-y1)*q12;
        
       
    }else if(y1==y2)
    {
        cv::Vec3b q11=data.at<cv::Vec3b>(y1, x1);
        cv::Vec3b q21=data.at<cv::Vec3b>(y1, x2);
        v=(x2-x)/(x2-x1)*q11+(x-x1)/(x2-x1)*q21;
    }else
    {
        cv::Vec3b q11=data.at<cv::Vec3b>(y1, x1);
        cv::Vec3b q12=data.at<cv::Vec3b>(y2, x1);
        cv::Vec3b q21=data.at<cv::Vec3b>(y1, x2);
        cv::Vec3b q22=data.at<cv::Vec3b>(y2, x2);
        cv::Vec3b vxy1=(x2-x)/(x2-x1)*q11+(x-x1)/(x2-x1)*q21;
        cv::Vec3b vxy2=(x2-x)/(x2-x1)*q12+(x-x1)/(x2-x1)*q22;
        v=(y2-y)/(y2-y1)*vxy1+(y-y1)/(y2-y1)*vxy2;
    }
    b=v[0];
    g=v[1];
    r=v[2];
}