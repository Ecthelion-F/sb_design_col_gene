//
// Created by Telperion on 2023/10/3.
//

#include "LineSets.h"

int LineSets::addLine(bool type) {
    all_lines_id.push_back(num_of_lines);
    if (BUS_LINE == type) bus_lines_id.push_back(num_of_lines);

    cl.push_back(0);
    num_of_lines++;

    return num_of_lines - 1;
}


void LineSets::setLinePass(int line_id, int arc_id) {
    line_pass_arc[line_id][arc_id] = true;
}

bool LineSets::miu(int line_id, int arc_id) {
    return line_pass_arc[line_id][arc_id];
}

std::vector<int> LineSets::getLines() {
    return all_lines_id;
}

std::vector<int> LineSets::getBusLines() {
    return bus_lines_id;
}

LineSets::LineSets(){
    memset(line_pass_arc, false, sizeof(line_pass_arc));
}

