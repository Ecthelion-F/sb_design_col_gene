//
// Created by Telperion on 2023/9/29.
//

#ifndef GUROBI_DEMO_ARCSETS_H
#define GUROBI_DEMO_ARCSETS_H

#include "UsefulConstants.h"


class Arc{
public:
    int id;
    int start, end, dis, max_freq;
    double velocity;
    bool type; // 0 for metro arc, 1 for bus arc
    Arc(int id, int start, int end, int dis, int max_freq, double v, bool type);
};

class ArcSets {
private:
    int num=0;
    std::vector<int> all_arcs_id;
    std::vector<int> bus_arcs_id;
    std::vector<Arc> arcs;
    std::map<odp, int> odp_to_arc_id;

public:
    int addArc(int start, int end, int dis, int freq, double v, bool type);
    std::vector<int> getArcs();
    std::vector<int> getBusArcs();
    int getArcStart(int id) {return arcs[id].start;}
    int getArcEnd(int id) {return arcs[id].end;}
    int getArcDis(int id);
    int getArcId(int start, int end);
    double getVelocity(int id);
    int getArcFreq(int id);
};




#endif //GUROBI_DEMO_ARCSETS_H
