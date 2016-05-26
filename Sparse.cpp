//
//  Sparse.cpp
//  Document
//
//  Created by  刘骥 on 16/4/29.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#include "Sparse.hpp"
#include "Utils.hpp"
Sparse::Sparse(const std::string&dir,const std::string&parFile,const std::string&visFile)
{
    
    loadImages(dir, parFile);
    loadVisData(dir,visFile);
    std::cerr<<"检测特征点："<<std::endl;
    for (Image &image:images) {
        image.detectFeatures();
    }
}
void Sparse::loadVisData(const std::string&dir,const std::string&visFile)
{
    std::cerr<<"加载可见性数据";
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
void Sparse::loadImages(const std::string&dir,const std::string&parFile)
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


void Sparse::buildPatches()
{
    
    std::cerr<<"开始重建"<<std::endl;
    
    for (Image &rimage:images) {
        std::cerr<<"重建图像"<<rimage.id<<",待计算特征点数为:"<<rimage.features.size()<<std::endl;
        if(rimage.nimages.size()<2)
            continue;
        
        clock_t start,finish;
        start=clock();
        
        for (const Feature &f1:rimage.features) {
            
            
            if(f1.isInEmptyCell())
            {
                int success=0,fail1=0,fail2=0,fail3=0;
                std::vector<Feature> features;
                f1.findFeatures(features);
                for_each(features.begin(), features.end(),
                         [&](Feature&f2){
                             utils::triangluate(f1, f2, f2.point4D);
                         }
                         );
                //                sort(features.begin(), features.end(), [&](const Feature&fa,const Feature&fb)->bool {
                //                    if (fa.getDistanceToCameraCenter(rimage.cameraCenter)<fb.getDistanceToCameraCenter(rimage.cameraCenter)) {
                //                        return true;
                //                    }
                //                    else
                //                    {
                //                        return false;
                //                    }
                //                });
                PPatch bestPatch;
                double bestCost=-1;
                for (const Feature &f2:features) {
                    
                    if(utils::cosangle(f2.point4D, rimage.cameraCenter, f2.image->cameraCenter)<0.6)
                    {
                        
                        fail1++;
                        continue;
                    }
                    PPatch p=PPatch(new Patch());
                    p->center=f2.point4D;
                    p->normal=rimage.cameraCenter-f2.point4D;
                    double n=norm(p->normal);

                    p->normal=p->normal/n;
                    p->rimage=&rimage;
                    p->updateImage(0.4, 0.4);
                    
                    if(p->timages.size()<=2)
                    {
                        fail2++;
                        continue;
                    }
                    
                    p->optimze();
                    
                    p->updateImage(0.4, 0.7);
                    //p->showResult();
                    if(p->timages.size()>=2)
                    {
                     /*   if (f1.x==424&&f1.y==339) {
                            std::cerr<<p->cost<<std::endl;
                            p->showResult();
                        }*/
                        success++;
                        if(p->cost>bestCost)
                        {
                            bestPatch=p;
                            bestCost=p->cost;
                        }
                    }else
                    {
                        fail3++;
                    }
                    
                }
                if (bestPatch.get()!=NULL) {
                    //这个特征点是明显的外点但是现在没办法检测
                    /*if (f1.x==439&&f1.y==290) {
                        bestPatch->showResult();
                    }*/
                    std::cerr<<"best:"<<bestPatch->center(0,0)<<" "<<bestPatch->center(1,0)<<" "<<bestPatch->center(2,0)<<std::endl;
                    patches.push_back(bestPatch);
                    
                    int pid=(int)patches.size()-1;
                    bestPatch->updateImageCell(pid);
                }
                
                //std::cerr<<"特征点位置：("<<f1.x<<","<<f1.y<<")，待计算特征点："<<features.size()<<"角度过大特征点："<<fail1<<"图像数过少特征点："<<fail2<<"优化后图像数过少特征点："<<fail3<<"成功的特征点："<<success<<std::endl;
                //输出特征点重建情况，
                std::cerr<<"特征点位置:("<<rimage.id<<","<<f1.x<<","<<f1.y<<"),total:"<<features.size()<<",fail:"<<(fail1+fail2+fail3)<<",success:"<<success<<std::endl;
            }
            
            
            
            
        }
        
        finish=clock();
        std::cerr<<"耗时"<<(double)(finish-start)/CLOCKS_PER_SEC<<"s"<<std::endl;
        savePatches("a.ply");
    }
    
}

void Sparse::savePatches(const std::string &fileName)
{
    std::ofstream fout(fileName);
    
    fout<<"ply"<<std::endl;
    fout<<"format ascii 1.0"<<std::endl;
    fout<<"element vertex "<<patches.size()<<std::endl;
    fout<<"property float x"<<std::endl;
    fout<<"property float y"<<std::endl;
    fout<<"property float z"<<std::endl;
    fout<<"property uchar diffuse_red"<<std::endl;
    fout<<"property uchar diffuse_green"<<std::endl;
    fout<<"property uchar diffuse_blue"<<std::endl;
    fout<<"end_header"<<std::endl;
    for (int i=0; i<patches.size(); i++) {
        Mat4x1 center=patches[i]->center;
        double r=0,g=0,b=0;
        for(Image *image:patches[i]->timages)
        {
            
            Mat3x1 x=image->project(center);
            cv::Mat patch;
            getRectSubPix(image->data, cv::Size(1,1), cv::Point2d(x(0,0),x(1,0)), patch);
            b+=patch.at<cv::Vec3b>(0, 0)[0];
            g+=patch.at<cv::Vec3b>(0, 0)[1];
            r+=patch.at<cv::Vec3b>(0, 0)[2];
        }
        r/=patches[i]->timages.size();
        g/=patches[i]->timages.size();
        b/=patches[i]->timages.size();
        fout<<center(0,0)<<" "<<center(1,0)<<" "<<center(2,0)<<" "<<(int)r<<" "<<(int)g<<" "<<(int)b<<std::endl;
    }
    fout.close();
    std::cerr<<"保存结束"<<std::endl;
}

