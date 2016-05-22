//
//  Patch.cpp
//  Document
//
//  Created by  刘骥 on 16/4/26.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#include "Patch.hpp"
#include "Utils.hpp"
 void Patch::updateImage(double alpha1,double alpha2)
{
    timages.clear();
    simages.clear();
    Tex pTex1;
    Mat4x1 pxaxis,pyaxis;
    getPAxes(pxaxis, pyaxis);
    rimage->grabTex(center,pxaxis,pyaxis,pTex1);
    
    for(Image *nimage:rimage->nimages)
    {
        if(utils::cosangle(center, rimage->cameraCenter, nimage->cameraCenter)<0.6)
           continue;
        Tex pTex2;
        nimage->grabTex(center, pxaxis, pyaxis, pTex2);
        double v=pTex1.ncc(pTex2);
        
        if (v>alpha1) {
            simages.push_back(nimage);
        }
        
        if(v>alpha2)
        {
            timages.push_back(nimage);
        }
        
    }
    
}
 void Patch::encode(double &depth, double &alpha, double &beta)const
{
    //detph：patch中心到相机的距离
    //alpha：patch法线与相机Y轴的夹角(-π/2,π/2)
    //beta：patch法线在XOZ平面上的投影与相机Z轴的夹角(-π/2,π/2)
    depth=rimage->getDistanceToCameraCenter(center);
    
    double fx=rimage->xaxis.dot(normal);
    double fy=rimage->yaxis.dot(normal);
    double fz=rimage->zaxis.dot(normal);
    alpha=asin(fy);
    double cosAlpha=cos(alpha);
    double sinBeta=fx/cosAlpha; //fx=sinβ*cosα
    double cosBeta=-fz/cosAlpha; //-fz=cosβ*cosα
    beta=acos(cosBeta);
    if (sinBeta<0) {
        beta=-beta;
    }
    //cout<<sin(beta)*cos(alpha)-fx<<endl;
    //cout<<sin(alpha)-fy<<endl;
    //cout<<-cos(beta)*cos(alpha)-fz<<endl;
    //cout<<depth<<" "<<alpha<<" "<<beta<<endl;
    
}
 void Patch::decode(double depth, double alpha, double beta)
{
   
    double fx=sin(beta)*cos(alpha);
    double fy=sin(alpha);
    double fz=-cos(beta)*cos(alpha);
    
    normal=fx*rimage->xaxis+fy*rimage->yaxis+fz*rimage->zaxis;
    normal(3,0)=0;
    center=rimage->cameraCenter+ray*depth;
}
 void Patch::getPAxes(Mat4x1 &pxaxis, Mat4x1 &pyaxis)const
{
    
    Image &image=*rimage;
    cv::Vec3d zaxis(normal(0,0),normal(1,0),normal(2,0));
    cv::Vec3d xaxis=cv::Vec3d(image.xaxis(0,0),image.xaxis(1,0),image.xaxis(2,0));
    cv::Vec3d yaxis=zaxis.cross(xaxis);
    yaxis=yaxis/norm(yaxis);
    xaxis=yaxis.cross(zaxis);
    double depth=norm(center-image.cameraCenter);
    double scale=2*depth/(image.kmat(0,0)+image.kmat(1,1));
    pxaxis(0,0)=xaxis[0];
    pxaxis(1,0)=xaxis[1];
    pxaxis(2,0)=xaxis[2];
    pxaxis(3,0)=0;
    pyaxis(0,0)=yaxis[0];
    pyaxis(1,0)=yaxis[1];
    pyaxis(2,0)=yaxis[2];
    pyaxis(3,0)=0;
    pxaxis=pxaxis*scale;
    pyaxis=pyaxis*scale;
    double xdis=norm(image.project(center+pxaxis)-image.project(center));
    double ydis=norm(image.project(center+pyaxis)-image.project(center));
    pxaxis=pxaxis/xdis;
    pyaxis=pyaxis/ydis;
    
}

 double Patch::averageCost()const
{
    if (timages.size()==0) {
        return -1;
    }
    Tex pTex1;
    Mat4x1 pxaxis,pyaxis;
    getPAxes(pxaxis, pyaxis);
    
    rimage->grabTex(center,pxaxis,pyaxis,pTex1);
   
    double sum=0;
    //TODO nimages还是timages？
    for(Image *timage:timages)
    {
        Tex pTex2;
        timage->grabTex(center, pxaxis, pyaxis, pTex2);
        sum+=pTex1.ncc(pTex2);
    }
   
    double ret=sum/timages.size();
    if(std::isnan(ret)||std::isinf(ret))
        return -1;
    return ret;
}
double optimizeFun(const std::vector<double> &x, std::vector<double> &grad, void *fdata)
{
   

    Patch p;
    Patch *oldP=(Patch*)fdata;
    p.timages=oldP->timages;
    p.rimage=oldP->rimage;
    p.ray=p.ray;
    p.decode(x[0], x[1], x[2]);
    
    double ret=p.averageCost();
    
    return ret;
    
}
void Patch::optimze()
{
    double depth,alpha,beta;
    ray=center-rimage->cameraCenter;
    ray=ray/norm(ray);
    encode(depth, alpha, beta);
    cost=averageCost();
    nlopt::opt opt(nlopt::LN_BOBYQA,3);
    opt.set_max_objective(optimizeFun, this);
    opt.set_xtol_rel(1.e-7);
    opt.set_maxeval(100);
    std::vector<double> x(3),lb(3),ub(3);
    lb[0]=-HUGE_VAL; //HUGE_VAL极大值
    lb[1]=-M_PI_2/3; //-pi/2
    lb[2]=-M_PI_2/3; //-pi/2
    ub[0]=HUGE_VAL;
    ub[1]=M_PI_2/3;
    ub[2]=M_PI_2/3;
    x[0]=depth;
    x[1]=alpha;
    x[2]=beta;
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
    bool success=false;
    double maxf=cost;
    try{
        nlopt::result result=opt.optimize(x, maxf);
        success = (result == nlopt::SUCCESS
                   || result == nlopt::STOPVAL_REACHED
                   || result == nlopt::FTOL_REACHED
                   || result == nlopt::XTOL_REACHED);
        //cout<<"maxf:"<<maxf<<endl;
    }catch(std::exception &e)
    {
        success = false;
    }
    if (success) {
        if(maxf>cost)
        {
            decode(x[0], x[1], x[2]);
            cost=maxf;
        }
    }
    
}
void Patch::updateImageCell(int pid)
{
    Mat3x1 coord=rimage->project(this->center);
    rimage->qt[coord(1,0)/2][coord(0,0)/2].insert(pid);
    

   for(Image *nimage:rimage->nimages)
    {
         coord=nimage->project(this->center);
        if(find(timages.begin(), timages.end(), nimage)!=timages.end())
        {
            nimage->qt[coord(1,0)/2][coord(0,0)/2].insert(pid);
        }else if(find(simages.begin(), simages.end(), nimage)!=simages.end()){
            
            nimage->qf[coord(1,0)/2][coord(0,0)/2].insert(pid);
        }
        
    }
}
void Patch::showResult()
{
    std::cout<<"timages:"<<timages.size()<<std::endl;
    //std::cout<<center<<std::endl;
    Image &image1=*rimage;
    
    cv::Mat im1,im2;
    image1.data.copyTo(im1);
    Mat3x1 coord=image1.project(center);
    cv::circle(im1, cv::Point2d(coord(0,0),coord(1,0)),5, cv::Scalar(0,0,255));
    cv::namedWindow("im1");
    cv::namedWindow("im2");
    cv::imshow("im1", im1);
    for (int i=0; i<timages.size(); i++) {
        Image &image2=*timages[i];
        std::cout<<utils::cosangle(center,image1.cameraCenter,image2.cameraCenter)<<std::endl;
        image2.data.copyTo(im2);
        coord=image2.project(center);
        cv::circle(im2, cv::Point2d(coord(0,0),coord(1,0)),5, cv::Scalar(0,0,255));
        cv::imshow("im2",im2);
        cv::waitKey();
        
    }

}
