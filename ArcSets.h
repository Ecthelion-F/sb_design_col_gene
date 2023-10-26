//
// Created by Telperion on 2023/9/29.
//

#ifndef GUROBI_DEMO_ARCSETS_H
#define GUROBI_DEMO_ARCSETS_H

#include "UsefulConstants.h"

class Arc{
public:
    int id;
    int start, end, max_freq;
    double dis, velocity;
    bool type; // 0 for metro arc, 1 for bus arc
    Arc(int id, int start, int end, double dis, int max_freq, double v, bool type);
};

class ArcSets {
private:
    int num=0;
    std::vector<int> all_arcs_id;
    std::vector<int> bus_arcs_id;
    std::vector<Arc> arcs;
    std::map<odp, int> odp_to_arc_id;
    const int INF = 1e9;

public:
    int min_id = INF, max_id = -INF;
    int addArc(int start, int end, double dis, int freq, double v, bool type);
    std::vector<int> getArcs();
    std::vector<int> getBusArcs();
    int getArcStart(int id) {return arcs[id].start;}
    int getArcEnd(int id) {return arcs[id].end;}
    double getArcDis(int id);
    int getArcId(int start, int end);
    double getVelocity(int id);
    int getArcFreq(int id);

    void readArcsFromFile(const std::string& filename);
};




#endif //GUROBI_DEMO_ARCSETS_H
