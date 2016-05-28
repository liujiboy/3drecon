//
//  main.cpp
//  Document
//
//  Created by  刘骥 on 16/4/26.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#include <stdio.h>
#include <iostream>
#include "Image.hpp"
#include "Patch.hpp"
#include "Feature.hpp"
#include <fstream>
#include <vector>
#include "Sparse.hpp"
using namespace std;

int main()
{
    //Sparse sparse("../../images/temple/temple_par.txt","../../images/temple/");
    
   // Sparse sparse("../../images/templeRing/","templeR_par.txt","vis.dat");
    Sparse sparse("/Users/liuji/Projects/3drecon/data/","templeR_par.txt","vis.dat",10);
   // Sparse sparse("/Users/liuji/Projects/doc/Furukawa2006/images/temple/","temple_par.txt","vis.dat",10);
   // Sparse sparse("../../images/templeSparseRing/templeSR_par.txt","../../images/templeSparseRing/");
    sparse.buildPatches();

}