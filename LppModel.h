//
// Created by Telperion on 2023/10/9.
//

#ifndef GUROBI_DEMO_LPPMODEL_H
#define GUROBI_DEMO_LPPMODEL_H


#include "gurobi_c++.h"
#include "UsefulConstants.h"
#include <vector>
#include <map>

using std::to_string, std::cout, std::endl;
using std::vector, std::map;


class LppModel {
private:
    int NUM_OF_NODES, MAX_LENGTH, ORIGIN = -1, DESTINATION = -1;
public:
    void setOrigin(int origin);

    void setDestination(int destination);

private:

    double bias = 0;

    void buildVars();
    void buildObj();
    void buildConstrains();

public:
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);
    vector< vector<double> > c_ij;
    map<int, GRBVar> u;
    map<pii, GRBVar> x;
    int num_of_vars=0, num_of_cons=0;

    LppModel(vector< vector<double> > &c, int max_length);

    void setBias(double b){bias = b;};
    void clear();
    void buildModel();
    double solve();

    vector<odp> getResult();

    void printResult();
};


#endif //GUROBI_DEMO_LPPMODEL_H
