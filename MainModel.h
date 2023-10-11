//
// Created by Telperion on 2023/10/9.
//

#ifndef GUROBI_DEMO_MAINMODEL_H
#define GUROBI_DEMO_MAINMODEL_H

#include "UsefulConstants.h"
#include "ArcSets.h"
#include "LineSets.h"
#include "ODSets.h"
#include "gurobi_c++.h"

using std::to_string, std::cout, std::endl;
using std::vector;
using std::pair, std::map;

const int INF = 1.1e9;

class MainModel {
private:
    double phi1 = 0.13, phi2 = 0.145, phi3 = 0.725;
    double lambda = 10;
    double alpha = 90.0;
public:
    int num_of_vars=0, num_of_cons=0;
    std::map<int, GRBVar> x;
    std::map<pii, GRBVar> y, z;

    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);


    void addVars(LineSets &lineSets, ODSets &odSets);
    void setObjective(LineSets &lineSets, ODSets &odSets);
    void addConstrs(ArcSets &arcSets, LineSets &lineSets, ODSets &odSets);
    vector< vector<double> > getDualX(ArcSets &arcSets);
    vector< vector<double> > getDualZ(ArcSets &arcSets, ODSets &odSets, int k);
    void toInt();
    void toCont();
    void optimize(){model.optimize();};

    void printResult(LineSets &lineSets, ODSets &odSets);


};


#endif //GUROBI_DEMO_MAINMODEL_H
