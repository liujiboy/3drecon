//
//  Sparse.hpp
//  Document
//
//  Created by  刘骥 on 16/4/29.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#ifndef Sparse_hpp
#define Sparse_hpp
#include <vector>
#include <opencv2/opencv.hpp>
#include "Image.hpp"
#include "Patch.hpp"
#include "Tex.hpp"
#include <fstream>
#include <string>

class Sparse{
private:
    std::vector<Image> images;
    std::vector<PPatch> patches;
    void loadImages(const std::string&dir,const std::string&parFile);
    void loadVisData(const std::string&dir,const std::string&visFile);
    void detectFeatures();
    int nThread;
public:
    Sparse(const std::string&dir,const std::string&parFile,const std::string&visFile,int n=20);
    void buildPatches();
    void savePatches(const std::string&fileName);
};
#endif /* Sparse_hpp */
