//
//  Sparse.cpp
//  Document
//
//  Created by  刘骥 on 16/4/29.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#include "Sparse.hpp"
#include "Utils.hpp"
#include<thread>
#include<queue>
#include <chrono>
Sparse::Sparse(const std::string&dir,const std::string&parFile,const std::string&visFile,int n):nThread(n)
{
    
    utils::loadImages(dir, parFile,images);
    utils::loadVisData(dir,visFile,images);
    detectFeatures();
}
void Sparse::detectFeatures()
{
    std::cerr<<"检测特征点："<<std::endl;
    std::vector<std::thread> threads;
    std::queue<Image*> imageQueue;
    std::mutex m;
    for (Image &image:images) {
        imageQueue.push(&image);
    }
    for(int i=0;i<nThread;i++)
    {
        threads.push_back(std::thread([&](){
            
            while(true)
            {
                Image *image=NULL;
                m.lock();
                if(!imageQueue.empty())
                {
                    image=imageQueue.front();
                    imageQueue.pop();
                    std::cerr<<"检测图像"<<image->id<<std::endl;
                    m.unlock();
                }else
                {
                    m.unlock();
                    break;
                }
                
                image->detectFeatures();
            }
        }));
    }
    for(auto &t:threads)
    {
        t.join();
    }

}
//void Sparse::loadVisData(const std::string&dir,const std::string&visFile)
//{
//    std::cerr<<"加载可见性数据";
//    std::ifstream vis((dir+visFile).c_str());
//    std::string header;
//    int num;
//    vis>>header>>num;
//    for (int i=0; i<num; i++) {
//        std::cerr<<"*";
//        int index1,index2,index2Size;
//        vis>>index1>>index2Size;
//        for(int j=0;j<index2Size;j++)
//        {
//            vis>>index2;
//            images[index1].nimages.push_back(&images[index2]);
//            Mat3x3 F;
//            utils::computeF(images[index1].pmat, images[index2].pmat, images[index1].cameraCenter, F);
//            images[index1].F.push_back(F);
//        }
//    }
//    std::cerr<<std::endl;
//    
//}
//void Sparse::loadImages(const std::string&dir,const std::string&parFile)
//{
//    std::cerr<<"加载图像中:";
//    std::ifstream fin((dir+parFile).c_str());
//    int n;
//    fin>>n;
//    //images.resize(n);
//    for(int i=0;i<n;i++)
//    {
//        std::cerr<<"*";
//        std::string name;
//        cv::Mat_<double> k(3,3);
//        cv::Mat_<double> rt(3,4);
//        fin>>name>>k(0,0)>>k(0,1)>>k(0,2)>>k(1,0)>>k(1,1)>>k(1,2)>>k(2,0)>>k(2,1)>>k(2,2)>>rt(0,0)>>rt(0,1)>>rt(0,2)>>rt(1,0)>>rt(1,1)>>rt(1,2)>>rt(2,0)>>rt(2,1)>>rt(2,2)>>rt(0,3)>>rt(1,3)>>rt(2,3);
//        images.push_back(Image(dir+name,i,k,rt));
//    }
//    std::cerr<<std::endl;
//    
//}


void Sparse::buildPatches()
{
    
    std::cerr<<"开始重建"<<std::endl;
    std::chrono::high_resolution_clock::time_point start,end;
    start= std::chrono::high_resolution_clock::now();
    std::vector<std::thread> threads;
    std::vector<Image*> shuffledImages;
    for (Image &image:images) {
        shuffledImages.push_back(&image);
    }
    std::random_shuffle(shuffledImages.begin(), shuffledImages.end());
    std::queue<Image*> imageQueue;
    std::mutex m;
    for (Image *image:shuffledImages) {
        imageQueue.push(image);
    }
    //random_shuffle(imageQueue.);
    for(int i=0;i<nThread;i++)
    {
        threads.push_back(std::thread([&](){
            while(true)
            {
                Image *rimage=NULL;
                m.lock();
                if(!imageQueue.empty())
                {
                    rimage=imageQueue.front();
                    imageQueue.pop();
                    std::cerr<<"重建图像"<<rimage->id<<",待计算特征点数为:"<<rimage->features.size()<<std::endl;
                    m.unlock();
                }else
                {
                    m.unlock();
                    break;
                }
                
                if(rimage->nimages.size()<2)
                    continue;
                for (const Feature &f1:rimage->features) {
                    
                    
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
                            
                            if(utils::cosangle(f2.point4D, rimage->cameraCenter, f2.image->cameraCenter)<0.6)
                            {
                                
                                fail1++;
                                continue;
                            }
                            PPatch p=PPatch(new Patch());
                            p->center=f2.point4D;
                            p->normal=rimage->cameraCenter-f2.point4D;
                            double n=norm(p->normal);
                            
                            p->normal=p->normal/n;
                            p->rimage=rimage;
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
                            //std::cerr<<"best:"<<bestPatch->center(0,0)<<" "<<bestPatch->center(1,0)<<" "<<bestPatch->center(2,0)<<std::endl;
                            m.lock();
                            int pid=(int)patches.size();
                            bestPatch->updateImageCell(pid);
                            patches.push_back(bestPatch);
                            m.unlock();
                            
                            
                        }
                        //输出特征点重建情况，
                        //std::cerr<<"特征点位置:("<<rimage->id<<","<<f1.x<<","<<f1.y<<"),total:"<<features.size()<<",fail:"<<(fail1+fail2+fail3)<<",success:"<<success<<std::endl;
                    }
                    
                    
                    
                    
                }
                
            }
        }));
    }
    for(auto &t:threads)
    {
        t.join();
    }
    end= std::chrono::high_resolution_clock::now();
    std::chrono::duration<double>  time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end-start);
    std::cerr<<"耗时"<<time_span.count()<<"s"<<std::endl;
    savePatches("a.ply");
}

void Sparse::savePatches(const std::string &fileName)
{
    utils::savePatches(patches, fileName);
}
void toString(const std::vector<std::vector<std::set<int> > >&q,int&count,std::string&str)
{
    count=0;
    std::stringstream ss;
    for (int i=0; i<q.size(); i++) {
        for(int j=0;j<q[i].size();j++)
        {
            if(q[i][j].size()>0)
            {
                ss<<i<<" "<<j<<" "<<q[i][j].size()<<" ";
                for (int id:q[i][j]) {
                    ss<<id<<" ";
                }
                ss<<std::endl;
                count++;
            }
        }
    }
    str=ss.str();

}
void Sparse::saveResult(const std::string&fileName)
{
    std::ofstream ofs(fileName.c_str());
    std::cerr<<"保存Qf和Qt"<<std::endl;
    ofs<<images.size()<<std::endl; //图像数量
   // ofs<<"**********Qf和Qt***************"<<std::endl;
    for (Image &image:images) {
        int count=0;
        std::string str;
        toString(image.qf,count,str);
        ofs<<count<<std::endl; //Qf
        ofs<<str;
        toString(image.qt,count,str);
        ofs<<count<<std::endl; //Qt
        ofs<<str;
    }
   // ofs<<"***********patches**************"<<std::endl;
    std::map<Image*,int> m;
    for(int i=0;i<images.size();i++)
    {
        m[&images[i]]=i;
    }
    ofs<<patches.size()<<std::endl; //patch数量
    for (PPatch p:patches) {
       // ofs<<"**********p***************"<<std::endl;
        ofs<<p->center(0,0)<<" "<<p->center(1,0)<<" "<<p->center(2,0)<<std::endl;
        ofs<<p->normal(0,0)<<" "<<p->normal(1,0)<<" "<<p->normal(2,0)<<std::endl;
        ofs<<p->ray(0,0)<<" "<<p->ray(1,0)<<" "<<p->ray(2,0)<<std::endl;
        ofs<<p->cost<<std::endl;
        ofs<<m[p->rimage]<<std::endl; //rimage index
        ofs<<p->simages.size()<<" ";
        for (Image *simage:p->simages) {
            ofs<<m[simage]<<" ";
        }
        ofs<<std::endl;
        ofs<<p->timages.size()<<" ";
        for (Image *timage:p->timages) {
            ofs<<m[timage]<<" ";
        }
        ofs<<std::endl;

    }
    ofs.close();
}

