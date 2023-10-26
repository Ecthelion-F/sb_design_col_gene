//
// Created by Telperion on 2023/9/29.
//

#include "ArcSets.h"


int ArcSets::addArc(int start, int end, double dis, int freq, double v, bool type) {
    Arc e = Arc(num++, start, end, dis, freq, v, type);
    arcs.push_back(e);
    if (BUS_ARC == type) bus_arcs_id.push_back(e.id);
    all_arcs_id.push_back(e.id);
    odp_to_arc_id[odp(start, end)] = e.id;

    if (min_id > start || min_id > end) min_id = start < end ? start : end;
    if (max_id < start || max_id < end) max_id = start > end ? start : end;

    return e.id;
}

std::vector<int> ArcSets::getArcs() {
    return all_arcs_id;
}

std::vector<int> ArcSets::getBusArcs() {
    return bus_arcs_id;
}

double ArcSets::getArcDis(int id) {
    return arcs[id].dis;
}

int ArcSets::getArcId(int start, int end) {
    return odp_to_arc_id[odp(start, end)];
}

double ArcSets::getVelocity(int id) {
    return arcs[id].velocity;
}

int ArcSets::getArcFreq(int id) {
    return arcs[id].max_freq;
}

void ArcSets::readArcsFromFile(const std::string& filename) {
    std::ifstream fin;
    int n;

    fin.open(filename);
    if(!fin.is_open()) {
        std::cout << "Error opening file" << std::endl;
        exit(1);
    }

    fin >> n;

    for (int i = 0; i < n; ++i){
        int start, end, freq, type;
        double dis, v;
        fin >> start >> end >> dis >> freq >> v >> type;
        addArc(start, end, dis, freq, v, type == 0 ? METRO_ARC : BUS_ARC);
    }

    std::cout << "[INFO] ArcSets: " << n << " arcs read from file " << filename << std::endl;
}

Arc::Arc(int id, int start, int end, double dis, int max_freq, double v, bool type) {
    this->id = id;
    this->start = start;
    this->end = end;
    this->dis = dis;
    this->max_freq = max_freq;
    this->velocity = v;
    this->type = type;
}
