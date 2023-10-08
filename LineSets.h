//
// Created by Telperion on 2023/10/3.
//

#ifndef GUROBI_DEMO_LINESETS_H
#define GUROBI_DEMO_LINESETS_H

#include "UsefulConstants.h"

class LineSets {
private:
    int num_of_lines = 0;
    std::vector<int> bus_lines_id;
    std::vector<int> all_lines_id;
    bool line_pass_arc[MAX_LINE_NUM][MAX_ARC_NUM]{};


public:
    LineSets();

    std::vector<double> cl;
    bool miu(int line_id, int arc_id);
    void setLinePass(int line_id, int arc_id);
    int addLine(bool type);

    std::vector<int> getLines();
    std::vector<int> getBusLines();
};


#endif //GUROBI_DEMO_LINESETS_H
