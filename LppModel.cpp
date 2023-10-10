//
// Created by Telperion on 2023/10/9.
//

#include "LppModel.h"

void LppModel::buildModel() {
    clear();
    buildVars();
    buildObj();
    buildConstrains();
}

LppModel::LppModel(vector<vector<double>> &c, int max_length, int ori, int des) {
    c_ij = c;
    MAX_LENGTH = max_length;
    ORIGIN = ori;
    DESTINATION = des;
    NUM_OF_NODES = (int)c_ij.size();
}

void LppModel::buildVars() {
    try{
        for (int i = 0; i < NUM_OF_NODES; ++i){
            u[i] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "u_" + to_string(i));
            num_of_vars++;
            for (int j = 0; j < NUM_OF_NODES; ++j){
                if (i != j){
                    x[pii(i, j)] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x_" + to_string(i) + "_" + to_string(j));
                    num_of_vars++;
                }
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


void LppModel::buildObj() {
    try{
        GRBLinExpr obj = 0;
        for (int i = 0; i < NUM_OF_NODES; ++i){
            for (int j = 0; j < NUM_OF_NODES; ++j){
                if (i != j){
                    obj += c_ij[i][j] * x[pii(i, j)];
                }
            }
        }
        model.setObjective(obj - bias, GRB_MAXIMIZE);
    } catch (GRBException &e){
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch (...){
        cout << "Exception during optimization" << endl;
    }
}


void LppModel::buildConstrains() {
    try{
        GRBLinExpr lhs = 0.0;
        for (auto iter = x.begin(); iter != x.end(); ++iter){
            lhs += iter->second;
        }
        model.addConstr(lhs <= MAX_LENGTH, "c_path_len");

        lhs = 0.0;
        for (int i = 0; i < NUM_OF_NODES; ++i){
            if (i != ORIGIN){
                lhs += x[pii(ORIGIN, i)];
            }
        }
        model.addConstr(lhs == 1.0, "origin_out_degree");

        lhs = 0.0;
        for (int i = 0; i < NUM_OF_NODES; ++i){
            if (i != DESTINATION){
                lhs += x[pii(i, DESTINATION)];
            }
        }
        model.addConstr(lhs == 1.0, "destination_in_degree");

        num_of_cons += 3;

        for (int i = 0; i < NUM_OF_NODES; ++i){
            GRBLinExpr lhsij = 0, lhsji = 0;
            for (int j = 0; j < NUM_OF_NODES; ++j){
                if (i != j){
                    lhsij += x[pii(i, j)];
                    lhsji += x[pii(j, i)];

                    model.addConstr(u[i] - u[j] + (NUM_OF_NODES - 1) * x[pii(i, j)] <= NUM_OF_NODES - 2,
                                    "mtz_" + to_string(i) + "_" + to_string(j));
                    num_of_cons++;
                }
            }
            if ((i != ORIGIN) && (i != DESTINATION)){
                model.addConstr(lhsij == lhsji, "equal_degree_" + to_string(i));
                model.addConstr(lhsij <= 1, "visit_once_" + to_string(i));
                num_of_cons += 2;
            }
        }


    } catch (GRBException &e){
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch (...){
        cout << "Exception during optimization" << endl;
    }

}


void LppModel::clear() {
    GRBVar* vars = model.getVars();
    GRBConstr* cons = model.getConstrs();
    for (int i = 0; i < num_of_vars; ++i) model.remove(vars[i]);
    for (int i = 0; i < num_of_cons; ++i) model.remove(cons[i]);
    model.update();
}


void LppModel::printResult() {
    for (auto iter = x.begin(); iter != x.end(); ++iter){
        if (iter->second.get(GRB_DoubleAttr_X) > 0){
            cout << iter->first.first << " -> " << iter->first.second << endl;
        }
    }
    cout << "Longest length : " << model.get(GRB_DoubleAttr_ObjVal) << endl;
}


void LppModel::solve() {
    model.set(GRB_IntParam_OutputFlag, 0);
    model.optimize();
    model.write("lpp.lp");
}
