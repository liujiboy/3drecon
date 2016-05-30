//
//  Patch.hpp
//  Document
//
//  Created by  刘骥 on 16/4/26.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#ifndef Patch_hpp
#define Patch_hpp
#include <opencv2/opencv.hpp>
#include<vector>
#include<nlopt.hpp>
#include<memory>
#include"Image.hpp"
#include "Matmxn.hpp"

class Patch{
private:
    double getUnit()const;
public:
    Mat4x1 center;
    Mat4x1 normal;
    Mat4x1 ray;
    Image* rimage; //reference image
    std::vector<Image*> simages; //S(p)
    std::vector<Image*> timages; //T(p)
    void encode(double &depth, double &alpha, double &beta)const;
    void decode(double depth, double alpha, double beta);
    void getPAxes( Mat4x1 &pxaxis, Mat4x1 &pyaxis)const ;
    double averageCost()const;
    void optimze();
    void updateImage(double alpha1,double alpha2);
    void updateImageCell(int pid);
    void showResult();
    double cost;
    double scale;
    void updateScale();
    bool isNeighbor(const Patch&p2)const;
    void intersect(const Image&image,double x,double y,Mat4x1&point)const ;
    bool isImageCellEmpty()const;

       //double cost(const Image&i,const Image&j);
    //void getXYAxis(const ImageSet&imageSet,Vec3d&xAxis,Vec3d&yAxis);
    //friend ostream&operator<<(ostream&os,const Patch&p);
};
typedef  std::shared_ptr<Patch> PPatch;
#endif /* Patch_hpp */
