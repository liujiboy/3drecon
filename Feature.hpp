//
//  Feature.hpp
//  Document
//
//  Created by  刘骥 on 16/4/28.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#ifndef Feature_hpp
#define Feature_hpp
#include <opencv2/opencv.hpp>
#include <vector>
#include "Image.hpp"
#include "Matmxn.hpp"
class Image;
class Feature{
public:
    Image *image;
    double x;
    double y;
    cv::Vec4d point4D;
    double getDistanceToCameraCenter(const cv::Vec4d&cameraCenter) const;//distance to camera center
    bool isInEmptyCell()const;
    Mat3x1 toHomogeneous()const;
    void findFeatures(std::vector<Feature>&featuresNearEpipolarLine)const;
};
#endif /* Feature_hpp */
