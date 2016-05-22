#ifndef UTILS_HPP
#define UTILS_HPP
#include<opencv2/opencv.hpp>
#include "Feature.hpp"
#include "Matmxn.hpp"

namespace utils{
    inline void triangluate(const Feature&f1,const Feature&f2,Mat4x1&point)
    {
        cv::Mat points4D;
        std::vector<cv::Point2d> points1;
        points1.push_back(cv::Point2d(f1.x,f1.y));
        std::vector<cv::Point2d> points2;
        points2.push_back(cv::Point2d(f2.x,f2.y));
        cv::triangulatePoints(f1.image->pmat, f2.image->pmat,points1,points2,points4D);
        point=points4D;
        point=point/point(3,0);
        point(3,0)=1;
    }
    inline double cosangle(const cv::Mat&point,const cv::Mat&c1,const cv::Mat&c2)
    {
        cv::Mat v1=point-c1;
        cv::Mat v2=point-c2;
        //double normV1=norm(v1);
        //double normV2=norm(v2);
//        if(normV1==0||normV2==0)
//            return -1;
        double ret=v1.dot(v2)/(norm(v1)*norm(v2));
        if(std::isnan(ret))
            return -1;
        return ret;
    }
//    inline void computeEpipolarLine(const Mat3x4 &p1,const Mat3x4 &p2, const Mat4x1&c1,const Feature &f, Mat3x1 &line)
//    {
//        //Mat3x4 p1=pmat;
//        //Mat4x1 c1=cameraCenter;
//        Mat3x3 pinvP1=p1.t()*(p1*p1.t()).inv();
//        Mat3x1 x;
//        x(0,0)=f.x;
//        x(1,0)=f.y;
//        x(2,0)=1;
//        line=(p2*c1).cross(p2*pinvP1)*x;
//    }
    inline void toCrossMat(double nx,double ny,double nz,Mat3x3& m)
    {
        m(0,0)=0;m(0,1)=-nz;m(0,2)=ny;
        m(1,0)=nz;m(1,1)=0;m(1,2)=-nx;
        m(2,0)=-ny;m(2,1)=nx;m(2,2)=0;
    }
    inline void computeF(const Mat3x4 &p1,const Mat3x4 &p2, const Mat4x1&c1,Mat3x3&F)
    {
        Mat3x3 pinvP1=p1.t()*(p1*p1.t()).inv();
        Mat3x1 e2=p2*c1;
        Mat3x3 m;
        toCrossMat(e2(0,0),e2(1,0),e2(2,0),m);
        F=m*p2*pinvP1;
    }
    
}
#endif
