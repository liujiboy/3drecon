#include "Image.hpp"
#include "Patch.hpp"
#include "Utils.hpp"
#include <vector>
#include<iostream>
#include<fstream>
#include <queue>
void loadPatches(const std::string&dir,const std::string&patchFile,std::vector<Image>&images,std::vector<PPatch>&patches)
{
    std::ifstream ifs(dir+patchFile);
    //load Qf,Qt
    int nImages;
    ifs>>nImages;
    for(int i=0;i<nImages;i++)
    {
        int nQf;
        ifs>>nQf;
        for(int j=0;j<nQf;j++)
        {
            int row,col,n;
            ifs>>row>>col>>n;
            for (int k=0; k<n; k++) {
                int id;
                ifs>>id;
                images[i].qf[row][col].insert(id);
            }
        }
        int nQt;
        ifs>>nQt;
        for(int j=0;j<nQt;j++)
        {
            int row,col,n;
            ifs>>row>>col>>n;
            for (int k=0; k<n; k++) {
                int id;
                ifs>>id;
                images[i].qt[row][col].insert(id);
            }
        }
    }
    int nPatches;
    ifs>>nPatches;
    for (int i=0; i<nPatches; i++) {
        PPatch p(new Patch());
        ifs>>p->center(0,0)>>p->center(1,0)>>p->center(2,0);
        p->center(3,0)=1;
        ifs>>p->normal(0,0)>>p->normal(1,0)>>p->normal(2,0);
        p->normal(3,0)=0;
        ifs>>p->ray(0,0)>>p->ray(1,0)>>p->ray(2,0);
        p->ray(3,0)=0;
        ifs>>p->cost;
        int rimage;
        ifs>>rimage;
        p->rimage=&images[rimage];
        int nSimages;
        ifs>>nSimages;
        for(int j=0;j<nSimages;j++)
        {
            int index;
            ifs>>index;
            p->simages.push_back(&images[index]);
        }
        int nTimages;
        ifs>>nTimages;
        for(int j=0;j<nTimages;j++)
        {
            int index;
            ifs>>index;
            p->timages.push_back(&images[index]);
        }
        //p.showResult();
        patches.push_back(p);
    }
}
void expandPatch(const PPatch&p,std::vector<PPatch>&patches,std::queue<PPatch>&patchQueue)
{
    std::vector<Image *> timages;
    //std::vector<Image *> timages=p.timages;
    timages.insert(timages.begin(), p->rimage);
    for (Image *timage:timages) {
        Mat3x1 point=timage->project(p->center);
        int col=point(0,0)/2;
        int row=point(1,0)/2;
        if(col<0||col>=timage->gridWidth||row<0||row>=timage->gridHeight)
            continue;
        for (int i=-1; i<=1; i++) {
            int x=col+i;
            if(x<0||x>=timage->gridWidth)
                continue;
            for(int j=-1;j<=1;j++)
            {
                if (i==0&&j==0) {
                    continue;
                }
                
                int y=row+j;
                if (y<0||y>=timage->gridHeight) {
                    continue;
                }
                if (timage->searched[y][x]) {
                    continue;
                }else
                {
                    timage->searched[y][x]=true;
                }
                if(!timage->qt[y][x].empty())
                    continue;
                if(!timage->qf[y][x].empty())
                {
                    bool flag=false;
                    for (int pid:timage->qf[y][x]) {
                        PPatch p2=patches[pid];
                        if(p->isNeighbor(*p2))
                        {
                            flag=true;
                            break;
                        }
                    }
                    if(flag)
                        continue;
                }
                //create new Patch;
                Mat4x1 newCenter;
                p->intersect(*timage, x*2+1, y*2+1, newCenter);
                //std::cout<<newCenter<<std::endl;
                PPatch newPatch(new Patch());
                *newPatch=*p;
                //std::cout<<newPatch->normal<<std::endl;
                newPatch->center=newCenter;
                newPatch->optimze();
                //std::cout<<newPatch->normal<<std::endl;
                newPatch->updateImage(0.4, 0.7);
                if(newPatch->timages.size()>=2&&newPatch->isImageCellEmpty())
                {
                    newPatch->updateScale();
                    int pid=patches.size();
                    newPatch->updateImageCell(pid);
                    patches.push_back(newPatch);
                    patchQueue.push(newPatch);
                 //  newPatch.showResult();
                }else
                {
                    //std::cout<<newPatch->isImageCellEmpty()<<std::endl;
                }
                //newPatch.showResult();
                //newPatch.u
                //std::cout<<p.center<<std::endl;
                //std::cout<<newCenter<<std::endl;
                //std::cout<<x*2+1<<","<<y*2+1<<std::endl;
                //std::cout<<timage->project(newCenter)<<std::endl;
                //std::cout<<"add new Patch:"<<timage->id<<","<<x<<","<<y<<std::endl;
            }
        }
        //std::cout<<std::endl;
        
    }
}
int main(int argc,char* argv[])
{
    std::vector<Image> images;
    std::vector<PPatch> patches;
    utils::loadImages("/Users/liuji/Projects/3drecon/data/","templeR_par.txt",images);
    utils::loadVisData("/Users/liuji/Projects/3drecon/data/","vis.dat",images);
    loadPatches("/Users/liuji/Projects/3drecon/data/","patches.txt",images,patches);
    for(PPatch p:patches)
    {
        p->updateScale();
       // std::cout<<p.scale<<std::endl;
    }
    std::queue<PPatch> patchQueue;
    for (PPatch p:patches) {
        patchQueue.push(p);
    }
    int n=1000;
    while (!patchQueue.empty()) {
        PPatch p=patchQueue.front();
        patchQueue.pop();
        std::cerr<<patchQueue.size()<<",";
        expandPatch(p,patches,patchQueue);
        std::cerr<<patchQueue.size()<<std::endl;
        if(patches.size()>n)
        {
            utils::savePatches(patches, "a.ply");
            n+=1000;
        }
    }
    utils::savePatches(patches, "a.ply");
}