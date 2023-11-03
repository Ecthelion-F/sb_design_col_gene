//
// Created by Telperion on 2023/10/3.
//

#ifndef GUROBI_DEMO_ODSETS_H
#define GUROBI_DEMO_ODSETS_H

#include "UsefulConstants.h"
#include "ArcSets.h"


class ODSets {
private:
    std::vector<int> id, demand;
    std::vector<int> num_of_paths;

    bool path_pass_arc[MAX_OD_PAIRS][MAX_PATH_NUM][MAX_ARC_NUM]{};

public:
    std::vector<int> origin, destination; // 方便外面一些地方抓数据
    ODSets();

    std::vector<std::vector<double> > ckp;

    int addOdPair(int ori, int dest, int dem);
    int addPathToOd(int od_id);
    void letPathPassArc(int od_id, int path_id, int arc_id); //k, p, e

    std::vector<int> getOds();
    int getPathNums(int od_id);
    int getDemand(int od_id);

    bool niu(int od_id, int path_id, int arc_id);

    void readOdsFromFile(const std::string& filename, ArcSets& arcSets);
};




#endif //GUROBI_DEMO_ODSETS_H
