//
// Created by Telperion on 2023/10/9.
//

#include "MainModel.h"

void MainModel::addVars(LineSets &lineSets, ODSets &odSets) {
    try{
        for (int i : lineSets.getBusLines()){
            x[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER, "x_" + to_string(i));
        }

        for (int k : odSets.getOds()){
            for (int p = 0; p < odSets.getPathNums(k); ++p){
                y[pii(k, p)] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "y_" + to_string(k) + "_" + to_string(p));
            }
        }

        for (int k : odSets.getOds()){
            for (int l : lineSets.getLines()){
                z[pii(k, l)] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z_" + to_string(k) + "_" + to_string(l));
            }
        }

        model.update();
    } catch (GRBException &e){
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch (...){
        cout << "Exception during optimization" << endl;
    }
}


void MainModel::toInt() {
    for (auto iter = x.begin(); iter != x.end(); ++iter){
        iter->second.set(GRB_CharAttr_VType, GRB_INTEGER);
    }
    for (auto iter = y.begin(); iter != y.end(); ++iter){
        iter->second.set(GRB_CharAttr_VType, GRB_BINARY);
    }
    for (auto iter = z.begin(); iter != z.end(); ++iter){
        iter->second.set(GRB_CharAttr_VType, GRB_BINARY);
    }
    model.update();
}


void MainModel::toCont() {
    for (auto iter = x.begin(); iter != x.end(); ++iter){
        iter->second.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
    }
    for (auto iter = y.begin(); iter != y.end(); ++iter){
        iter->second.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
    }
    for (auto iter = z.begin(); iter != z.end(); ++iter){
        iter->second.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
    }
    model.update();

}

void MainModel::setObjective(LineSets &lineSets, ODSets &odSets) {
    try {
        GRBLinExpr obj = GRBLinExpr();
        for (int i: lineSets.getBusLines()) {
            obj += phi1 * lineSets.cl[i] * x[i];
        }
        for (int k: odSets.getOds()) {
            for (int p = 0; p < odSets.getPathNums(k); ++p) {
                obj += phi2 * odSets.ckp[k][p] * odSets.getDemand(k) * y[pii(k, p)];
            }
        }
        for (int k: odSets.getOds()) {
            for (int l: lineSets.getLines()) {
                obj += phi3 * odSets.getDemand(k) * z[pii(k, l)];
            }
        }
        model.setObjective(obj, GRB_MINIMIZE);
    } catch (GRBException &e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch (...) {
        cout << "Exception during optimization" << endl;
    }
}


void MainModel::addConstrs(ArcSets &arcSets, LineSets &lineSets, ODSets &odSets) {
    try{
        // 约束组2
        for (int k : odSets.getOds()){
            GRBLinExpr lhs = GRBLinExpr();
            for (int p = 0; p < odSets.getPathNums(k); ++p){
                lhs += y[pii(k, p)];
            }
            model.addConstr(lhs == 1.0, "c2_" + to_string(k));
        }

        // 约束组3
        for (int k : odSets.getOds()){
            for (int p = 0; p < odSets.getPathNums(k); ++p){
                for (int e : arcSets.getArcs()){
                    GRBLinExpr lhs = GRBLinExpr();
                    for (int l : lineSets.getLines()){
                        lhs += lineSets.miu(l, e) * z[pii(k, l)];
                    }
                    lhs -= odSets.niu(k, p, e) * y[pii(k, p)];
                    model.addConstr(lhs >= 0.0, "c3_" + to_string(k) + "_" + to_string(p) + "_" + to_string(e));
                }
            }
        }

        // 约束组4
        for (int k : odSets.getOds()){
            for (int l : lineSets.getBusLines()){
                model.addConstr(x[l] - z[pii(k, l)] >= 0.0, "c4_" + to_string(k) + "_" + to_string(l));
            }
        }

        // 约束组5
        for (int e : arcSets.getBusArcs()){
            GRBLinExpr lhs = GRBLinExpr();
            for (int l : lineSets.getBusLines()){
                lhs += lambda * lineSets.miu(l, e) * x[l];
            }
            for (int k : odSets.getOds()){
                for (int p = 0; p < odSets.getPathNums(k); ++p){
                    lhs -= odSets.niu(k, p, e) * y[pii(k, p)];
                }
            }
            model.addConstr(lhs >= 0.0, "c5_" + to_string(e));
        }

        // 约束组6
        for (int e : arcSets.getBusArcs()){
            GRBLinExpr lhs = GRBLinExpr();
            for (int l : lineSets.getBusLines()){
                lhs += lineSets.miu(l, e) * x[l];
            }
            model.addConstr(lhs <= arcSets.getArcFreq(e), "c6_" + to_string(e));
        }
    } catch (GRBException &e){
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch (...){
        cout << "Exception during optimization" << endl;
    }

}