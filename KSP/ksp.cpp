//
// Created by Telperion on 2023/10/12.
//
// 先把k短路算出来待会再说路线的问题救命啊啊啊啊啊

#include<iostream>
#include<fstream>
#include<queue>
#include<vector>

using namespace std;

const int MAX_N = 200;

class edge{
public:
    int v;
    double dis;

    edge(int v, double dis): v(v), dis(dis){}

    bool operator < (const edge& a) const{
        return dis > a.dis;
    }
};

class info{
public:
    int v;
    double dis;
    int pre_v, pre_k;
    info(int v, double dis, int pre_v, int pre_k): v(v), dis(dis), pre_v(pre_v), pre_k(pre_k){}
    bool operator < (const info& a) const{
        return dis > a.dis;
    }
};

int main(){
    vector<edge> map[MAX_N];

    ifstream fin("arcSetsSetting.in");
    int n;
    fin >> n;
    for(int i = 0; i < n; ++i){
        int start, end, dis, max_freq, type;
        double velocity;
        fin >> start >> end >> dis >> max_freq >> velocity >> type;
        map[start].emplace_back(end, dis / velocity);
    }

    priority_queue<info> q;
    vector<info> dist[MAX_N];

    int s = 1;
    q.emplace(s, 0, 0, 0);
    while(!q.empty()){
        info e = q.top();
        q.pop();
        if(dist[e.v].size() >= 3) continue;

        dist[e.v].push_back(e);
        cout << endl << "----now find " << dist[e.v].size()-1 << "th path for " << e.v << " is " << e.dis
        << " from " << e.pre_v << "'s " << e.pre_k << "th path" << endl;

        for(auto i : map[e.v]){
            q.emplace(i.v, e.dis + i.dis, e.v, dist[e.v].size() - 1);
            cout << "extend to " << i.v << " with dis " << e.dis + i.dis << " using " << dist[e.v].size() - 1
            << "th path to " << e.v << endl;
        }
    }

    for (int i = 0; i < 5; ++i){
        cout << i << " : " << endl;
        for (auto j : dist[i]){
            cout << j.dis << ": ";
            info k = j;
            while(k.v != s){
                cout << k.v << "-";
                k = dist[k.pre_v][k.pre_k];
            }
            cout << s << endl;
//            cout << j.v << " " << j.dis << " " << j.pre_v << " " << j.pre_k << endl;
        }
        cout << endl;
    }

    return 0;
}