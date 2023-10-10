#include <iostream>

#include "MainModel.h"

using namespace std;

int main(int argc, char *argv[])
{
    ArcSets arcSets;
    LineSets lineSets;
    ODSets odSets;

    /*-------------------------------------------模型需要的各种参数------------------------------------------------------*/
    {
    // 把 arc 加进去
    arcSets.addArc(1, 3, 1, 10, 1.0, METRO_ARC);
    arcSets.addArc(3, 4, 1, 10, 1.0, METRO_ARC);
    arcSets.addArc(2, 3, 1, 10, 1.0, METRO_ARC);
    arcSets.addArc(2, 5, 9, 18, 1.0, BUS_ARC);
    arcSets.addArc(4, 5, 8, 1000, 1.0, BUS_ARC);


    // 加入空的 line
    for (int i = 0; i < 2; ++i) lineSets.addLine(BUS_LINE);
    for (int i = 0; i < 2; ++i) lineSets.addLine(METRO_LINE);

    // 配置各 line 经过的 arc
    lineSets.setLinePass(0, 4);
    lineSets.setLinePass(1, 3);
    lineSets.setLinePass(2, arcSets.getArcId(1, 3));
    lineSets.setLinePass(2, 1);
    lineSets.setLinePass(3, 2);
    lineSets.setLinePass(3, 1);


    // 加入 OD
    odSets.addOdPair(1, 5, 127);
    odSets.addOdPair(2, 5, 319);

    // 给各 OD 加入 path
    odSets.addPathToOd(0);
    odSets.addPathToOd(1);
    odSets.addPathToOd(1);

    // 配置各 OD 的 path 经过的 arc
    odSets.letPathPassArc(0, 0, arcSets.getArcId(1, 3));
    odSets.letPathPassArc(0, 0, arcSets.getArcId(3, 4));
    odSets.letPathPassArc(0, 0, arcSets.getArcId(4, 5));
    odSets.letPathPassArc(1, 0, arcSets.getArcId(2, 3));
    odSets.letPathPassArc(1, 0, arcSets.getArcId(3, 4));
    odSets.letPathPassArc(1, 0, arcSets.getArcId(4, 5));
    odSets.letPathPassArc(1, 1, arcSets.getArcId(2, 5));


    // 计算c_l
    const double ALPHA = 90.0;
    for (int i: lineSets.getLines()) {
        for (int j: arcSets.getBusArcs()) {
            lineSets.cl[i] += ALPHA * (double) lineSets.miu(i, j) * (double) arcSets.getArcDis(j);
        }
    }

    // 计算c_{k,p}
    for (int k: odSets.getOds()) {
        vector<double> ckp;
        for (int p = 0; p < odSets.getPathNums(k); ++p) {
            double tmp = 0.0;
            cout << sizeof(tmp);
            for (int e: arcSets.getArcs()) {
                tmp += (double)arcSets.getArcDis(e) * odSets.niu(k, p, e) / arcSets.getVelocity(e);
            }
            ckp.push_back(tmp);
        }
        odSets.ckp.push_back(ckp);
    }
}


    /*--------------------------------------------下面开始搭模型--------------------------------------------------------*/
    try{
        MainModel m = MainModel();
        m.addVars(lineSets, odSets);
//        m.toCont();
//        m.toInt();
        m.setObjective(lineSets, odSets);
        m.addConstrs(arcSets, lineSets, odSets);

        m.optimize();

        map<int, GRBVar> x = m.x;
        map<pii, GRBVar> y = m.y, z = m.z;

        // 输出结果
        cout << "Obj: " << m.model.get(GRB_DoubleAttr_ObjVal) << endl;
        for (int i : lineSets.getBusLines()){
            cout << "x_" << i << " = " << x[i].get(GRB_DoubleAttr_X) << endl;
        }
        for (int k : odSets.getOds()){
            for (int p = 0; p < odSets.getPathNums(k); ++p){
                cout << "y_" << k << "_" << p << " = " << y[pii(k, p)].get(GRB_DoubleAttr_X) << endl;
            }
        }
        for (int k : odSets.getOds()){
            for (int l : lineSets.getLines()){
                cout << "z_" << k << "_" << l << " = " << z[pii(k, l)].get(GRB_DoubleAttr_X) << endl;
            }
        }

        vector< vector<double> > vet = m.getDualX(arcSets);

        for (auto & i : vet){
            for (double j : i){
                cout << j << " ";
            }
            cout << endl;
        }

    } catch (GRBException &e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch (...) {
        cout << "Exception during optimization" << endl;
    }

    return 0;
}