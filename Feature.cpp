//
//  Feature.cpp
//  Document
//
//  Created by  刘骥 on 16/4/28.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#include "Feature.hpp"
#include "Utils.hpp"
bool Feature::isInEmptyCell()const
{
    int row=y/2;
    int col=x/2;
    //image->cellLock[row][col]->lock();
    bool ret=image->qt[row][col].empty();
    //image->cellLock[row][col]->unlock();
    return ret;
    //return image->qt[y/2][x/2].empty()&&image->qf[y/2][x/2].empty();
}
Mat3x1 Feature::toHomogeneous()const
{
    Mat3x1 m;
    m(0,0)=x;
    m(1,0)=y;
    m(2,0)=1;
    return m;
}
double Feature::getDistanceToCameraCenter(const cv::Vec4d &cameraCenter)const
{
    return norm(cameraCenter-point4D);
}


void Feature::findFeatures(std::vector<Feature>&featuresNearEpipolarLine)const
{
    
    for (int i=0;i<image->nimages.size();i++) {
        Image *nimage=image->nimages[i];
        Mat3x3 F=image->F[i];
        Mat3x1 x1;
        x1(0,0)=x;
        x1(1,0)=y;
        x1(2,0)=1;
        Mat3x1 line=F*x1;
        double a=line(0,0);
        double b=line(1,0);
        double c=line(2,0);
        //cout<<nimage->features.size()<<endl;
        for (const Feature &f:nimage->features) {
            if(f.isInEmptyCell())
            {
            double d=std::abs(a*f.x+b*f.y+c)/sqrt(a*a+b*b);
            if(d<=2)
            {
                featuresNearEpipolarLine.push_back(f);
            }
            }
        }
        
    }
}
