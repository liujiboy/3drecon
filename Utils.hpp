#ifndef UTILS_HPP
#define UTILS_HPP
#include<opencv2/opencv.hpp>
#include "Feature.hpp"
#include "Matmxn.hpp"
#include "Patch.hpp"
#include <fstream>
namespace utils{
    inline void triangluate(const Feature&f1,const Feature&f2,cv::Vec4d&point)
    {
        cv::Mat points4D;
        std::vector<cv::Point2d> points1;
        points1.push_back(cv::Point2d(f1.x,f1.y));
        std::vector<cv::Point2d> points2;
        points2.push_back(cv::Point2d(f2.x,f2.y));
        cv::triangulatePoints(f1.image->pmat, f2.image->pmat,points1,points2,points4D);
        double w=points4D.at<double>(3,0);
        point[0]=points4D.at<double>(0,0)/w;
        point[1]=points4D.at<double>(1,0)/w;
        point[2]=points4D.at<double>(2,0)/w;
        point[3]=1;
    }
    inline double cosangle(const cv::Vec4d&point,const cv::Vec4d&c1,const cv::Vec4d&c2)
    {
        cv::Vec4d v1=point-c1;
        cv::Vec4d v2=point-c2;
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
    inline cv::Mat pinv(const cv::Mat&m)
    {
        return m.t()*(m*m.t()).inv();
    }
    inline void computeF(const Mat3x4 &p1,const Mat3x4 &p2, const cv::Vec4d &c1,Mat3x3&F)
    {
        //Mat3x3 pinvP1=p1.t()*(p1*p1.t()).inv();
        cv::Mat pinvP1=pinv(p1);
        cv::Vec4d v1(p2(0,0),p2(0,1),p2(0,2),p2(0,3));
        cv::Vec4d v2(p2(1,0),p2(1,1),p2(1,2),p2(1,3));
        cv::Vec4d v3(p2(2,0),p2(2,1),p2(2,2),p2(2,3));
        cv::Vec3d e2;
        e2[0]=v1.dot(c1);
        e2[1]=v2.dot(c1);
        e2[2]=v3.dot(c1);
        Mat3x3 m;
        toCrossMat(e2[0],e2[1],e2[2],m);
        F=m*p2*pinvP1;
    }
    inline void loadImages(const std::string&dir,const std::string&parFile, std::vector<Image>&images)
    {
        std::cerr<<"加载图像中:";
        std::ifstream fin((dir+parFile).c_str());
        int n;
        fin>>n;
        //images.resize(n);
        for(int i=0;i<n;i++)
        {
            std::cerr<<"*";
            std::string name;
            cv::Mat_<double> k(3,3);
            cv::Mat_<double> rt(3,4);
            fin>>name>>k(0,0)>>k(0,1)>>k(0,2)>>k(1,0)>>k(1,1)>>k(1,2)>>k(2,0)>>k(2,1)>>k(2,2)>>rt(0,0)>>rt(0,1)>>rt(0,2)>>rt(1,0)>>rt(1,1)>>rt(1,2)>>rt(2,0)>>rt(2,1)>>rt(2,2)>>rt(0,3)>>rt(1,3)>>rt(2,3);
            images.push_back(Image(dir+name,i,k,rt));
        }
        std::cerr<<std::endl;
        
    }
    inline void loadVisData(const std::string&dir,const std::string&visFile,std::vector<Image>&images)
    {
        std::cerr<<"加载可见性数据:";
        std::ifstream vis((dir+visFile).c_str());
        std::string header;
        int num;
        vis>>header>>num;
        for (int i=0; i<num; i++) {
            std::cerr<<"*";
            int index1,index2,index2Size;
            vis>>index1>>index2Size;
            for(int j=0;j<index2Size;j++)
            {
                vis>>index2;
                images[index1].nimages.push_back(&images[index2]);
                Mat3x3 F;
                utils::computeF(images[index1].pmat, images[index2].pmat, images[index1].cameraCenter, F);
                images[index1].F.push_back(F);
            }
        }
        std::cerr<<std::endl;
        
    }
    inline void savePatches(const std::vector<PPatch>&patches,const std::string &fileName)
    {
        std::stringstream ss;
        int count=0;
        for (int i=0; i<patches.size(); i++) {
            cv::Vec4d center=patches[i]->center;
            cv::Vec4d normal=patches[i]->normal;
            double r=0,g=0,b=0;
            for(Image *image:patches[i]->timages)
            {
                
                cv::Vec3d x=image->project(center);
                cv::Mat patch;
                getRectSubPix(image->data, cv::Size(1,1), cv::Point2d(x[0],x[1]), patch);
                b+=patch.at<cv::Vec3b>(0, 0)[0];
                g+=patch.at<cv::Vec3b>(0, 0)[1];
                r+=patch.at<cv::Vec3b>(0, 0)[2];
            }
            r/=patches[i]->timages.size();
            g/=patches[i]->timages.size();
            b/=patches[i]->timages.size();
            if((r+g+b)/3>50)
            {
            ss<<center[0]<<" "<<center[1]<<" "<<center[2]<<" "<<normal[0]<<" "<<normal[1]<<" "<<normal[2]<<" "<<(int)r<<" "<<(int)g<<" "<<(int)b<<std::endl;
                count++;
            }
        }
        std::ofstream fout(fileName);
        fout<<"ply"<<std::endl;
        fout<<"format ascii 1.0"<<std::endl;
        fout<<"element vertex "<<count<<std::endl;
        fout<<"property float x"<<std::endl;
        fout<<"property float y"<<std::endl;
        fout<<"property float z"<<std::endl;
        fout<<"property float nx"<<std::endl;
        fout<<"property float ny"<<std::endl;
        fout<<"property float nz"<<std::endl;
        fout<<"property uchar diffuse_red"<<std::endl;
        fout<<"property uchar diffuse_green"<<std::endl;
        fout<<"property uchar diffuse_blue"<<std::endl;
        fout<<"end_header"<<std::endl;
        fout<<ss.str();
        fout.close();
        std::cerr<<"保存结束"<<std::endl;
    }
}
#endif
