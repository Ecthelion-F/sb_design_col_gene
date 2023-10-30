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

void ODSets::readOdsFromFile(const std::string &filename, ArcSets &arcSets) {
    std::ifstream fin;
    fin.open(filename);
    if (!fin){
        std::cout << "Error opening file" << std::endl;
        exit(1);
    }

    int n;
    fin >> n;
    for (int i = 0; i < n; ++i){
        int ori, dest, dem, paths;
        fin >> ori >> dest >> dem >> paths;
        int od_id = addOdPair(ori, dest, dem);

        for (int j = 0; j < paths; ++j){
            int num_of_arcs;
            fin >> num_of_arcs;
            int path_id = addPathToOd(od_id);

            for (int k = 0; k < num_of_arcs; ++k){
                int start, end;
                fin >> start >> end;
                letPathPassArc(od_id, path_id, arcSets.getArcId(start, end));
            }
        }
    }

    std::cout << "[INFO] ODSets: " << n << " O-D pairs read from file " << filename << std::endl;
}
