//
// Created by Telperion on 2023/10/3.
//

#ifndef GUROBI_DEMO_USEFULCONSTANTS_H
#define GUROBI_DEMO_USEFULCONSTANTS_H

#include<iostream>
#include<fstream>
#include<map>
#include<vector>
#include <string>

typedef std::pair<int, int> odp;
typedef std::pair<int, int> pii;

const double phi1 = 0.13, phi2 = 0.145, phi3 = 0.725;
const double lambda = 80;
const double alpha = 0.09; //90块一公里的车车好像也不是很过分？

#define MAX_OD_PAIRS 160
#define MAX_PATH_NUM 5
#define MAX_LINE_NUM 300
#define MAX_ARC_NUM 180

#define BUS_ARC 1
#define METRO_ARC 0

#define BUS_LINE 1
#define METRO_LINE 0

#endif //GUROBI_DEMO_USEFULCONSTANTS_H
