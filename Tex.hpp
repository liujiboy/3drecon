//
//  Tex.hpp
//  Document
//
//  Created by  刘骥 on 16/5/5.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#ifndef Tex_hpp
#define Tex_hpp
#include <vector>
#include <opencv2/opencv.hpp>
#include <memory>

class Tex{
public:
    Tex(int size=7);
    std::vector<double> values;
    //double size;
    double ncc(const Tex&other)const;
    //void getMinMaxXMinMaxY(int&minX,int&maxX,int&minY,int&maxY);
    //void updateCell(int id,vector<vector<set<int> > >&cell);
};
#endif /* Tex_hpp */
