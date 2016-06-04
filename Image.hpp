//
//  Image.hpp
//  Document
//
//  Created by  刘骥 on 16/4/28.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#ifndef Image_hpp
#define Image_hpp
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <vector>
#include "Feature.hpp"
#include "Tex.hpp"
#include "Matmxn.hpp"
#include<thread>
#include<memory>
class Feature;
class Image{
public:
    Image(const std::string&fileName,int i,const cv::Mat_<double>& k,const cv::Mat_<double>& rt);
    int id;
    cv::Mat data;
    Mat3x3 kmat;
    Mat3x4 pmat;
    cv::Vec4d pvec[3];
    Mat3x4 rtmat;
    Mat3x3 rmat;
    Mat3x1 tmat;
    cv::Vec4d cameraCenter;
    cv::Vec4d  xaxis;
    cv::Vec4d  yaxis;
    cv::Vec4d  zaxis;
    int gridWidth;
    int gridHeight;
    std::vector<std::vector<std::set<int> > > qf; //Qf(i,j)
    
    std::vector<std::vector<std::set<int> > > qt; //Qt(i,j)
    std::vector<std::vector<bool > > searched;
   // std::vector<std::vector<std::shared_ptr<std::mutex>> > cellLock;
    std::vector<std::vector<int> > depth; //depth(i,j)
    std::vector<Feature> features;
    cv::Mat descriptors;
    std::vector<Image*> nimages;
    std::vector<Mat3x3> F;
    void detectFeatures();
    bool isInEmptyCell(const Feature&f)const;
    //void findFeatures(const Feature &f1,vector<Feature>&featuresNearEpipolarLine)const;
    double getDistanceToCameraCenter(const cv::Vec4d &point)const;
    cv::Vec3d project(const cv::Vec4d &point)const;
    void grabTex(const cv::Vec4d coord,const cv::Vec4d&pxaxis,const cv::Vec4d&pyaxis,Tex&pTex)const;
    void getColor(double x,double y,double&r,double&g,double&b)const;
};
#endif /* Image_hpp */
