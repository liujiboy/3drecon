//
//  Matmxn.hpp
//  Document
//
//  Created by  刘骥 on 16/5/3.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#ifndef Matmxn_hpp
#define Matmxn_hpp

#include <opencv2/opencv.hpp>

template <int m,int n>
class Matmxn:public cv::Mat_<double>{
public:
    Matmxn():Mat_<double>(m,n)
    {
        
    }
    Matmxn(const Mat& mat):Mat_<double>(mat)
    {
        
    }
    Matmxn(const cv::MatExpr& e):Mat_<double>(e)
    {
        
    }

    Matmxn&operator=(const Mat&mat)
    {
        Mat_<double>::operator=(mat);
        return *this;
    }
    Matmxn&operator=(const Matmxn&mat)
    {
        Mat_<double>::operator=(mat);
        return *this;
    }
     Matmxn& operator = (const cv::MatExpr& e)
    {
        Mat_<double>::operator=(e);
        return *this;
    }
};
typedef Matmxn<3, 1> Mat3x1;
typedef Matmxn<4, 1> Mat4x1;
typedef Matmxn<3, 4> Mat3x4;
typedef Matmxn<3, 3> Mat3x3;

#endif /* Matmxn_hpp */
