//
// Created by Telperion on 2023/10/3.
//

#include "ODSets.h"


int ODSets::addOdPair(int ori, int dest, int d) {
    id.push_back(id.size());

    origin.push_back(ori);
    destination.push_back(dest);

    num_of_paths.push_back(0);
    demand.push_back(d);

    return id.size() - 1;
}

int ODSets::addPathToOd(int od_id) {
    return num_of_paths[od_id]++;
}

ODSets::ODSets() {
    memset(path_pass_arc, false, sizeof(path_pass_arc));
}

void ODSets::letPathPassArc(int od_id, int path_id, int arc_id) {
    path_pass_arc[od_id][path_id][arc_id] = true;
}

bool ODSets::niu(int od_id, int path_id, int arc_id) {
    return path_pass_arc[od_id][path_id][arc_id];
}

std::vector<int> ODSets::getOds() {
    return id;
}

int ODSets::getPathNums(int od_id) {
    return num_of_paths[od_id];
}

int ODSets::getDemand(int od_id) {
    return demand[od_id];
}
