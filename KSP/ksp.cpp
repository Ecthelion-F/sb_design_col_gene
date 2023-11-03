//
// Created by Telperion on 2023/10/12.
//

#include<iostream>
#include<fstream>
#include<queue>
#include<vector>
#include<stack>
#include <string>

using std::to_string;

const bool OUTPUT_YES = true;
const bool OUTPUT_NO = false;

using namespace std;

const int MAX_N = 200;

class edge{
public:
    int v;
    double dis;
    edge(int v, double dis): v(v), dis(dis){}
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

vector<edge> map[MAX_N];
vector<edge> metro_map[MAX_N];

vector<vector<info>> ksp(int start, int MAX_NODE_ID, int path_num, bool OUTPUT_FLAG, vector<edge> map_used[]);

int main(){

    ifstream fin("arcSetsSetting.in");
    int n, path_num, MAX_NODE_ID = 0; // 边总数，最大节点编号
    fin >> n;
    for(int i = 0; i < n; ++i){
        int start, end, max_freq, type;
        double dis, velocity;
        fin >> start >> end >> dis >> max_freq >> velocity >> type;

//        看我给每段公交都加两分钟惩罚你们一个个的还全都跑公交吗
        if (type == 1) map[start].emplace_back(end, dis / velocity + 2);
        else map[start].emplace_back(end, dis / velocity);

        if (type == 0) metro_map[start].emplace_back(end, dis / velocity);

        if (start > MAX_NODE_ID) MAX_NODE_ID = start;
        if (end > MAX_NODE_ID) MAX_NODE_ID = end;
    }
    fin.close();

    metro_map[0].emplace_back(1, 2); // tmp:只针对特例

    ofstream out("odSetsSetting.out");
    fin.open("odLists.in");
    fin >> n >> path_num;
    out << n << endl;
    for(int i = 0; i < n; ++i){
        int start, end, demand;
        fin >> start >> end >> demand;
        out << start << " " << end << " " << demand / 24 + 1 <<  " "; //需求量调到人数/小时才比较对.....
        cout << "\n from " << start << " to " << end << endl;

        vector<vector<info>> ans = ksp(start, MAX_NODE_ID, path_num, OUTPUT_NO, map);
        vector<vector<info>> ans_metro = ksp(start, MAX_NODE_ID, path_num, OUTPUT_NO, metro_map);

        stack<int> path;
        int real_path_num = 0;
        string str;
        for (auto j: ans[end]) {
            if (real_path_num >= path_num) break;
            info k = j;
            while (k.v != start) {
                if ((!path.empty()) && (k.v == end)) {
                    cout << "loop" << endl;
                    while (!path.empty()) path.pop();
                    break;
                }
                path.push(k.v);
                k = ans[k.pre_v][k.pre_k];
            }
            if ((!path.empty()) && (real_path_num < path_num)) {
                str += to_string(path.size()) + "\n";
                real_path_num++;
                int now = start, next;
                while (!path.empty()) {
                    next = path.top();
                    cout << now << " -> " << next << " ";
                    str += to_string(now) + " " + to_string(next) + "\n";
                    now = next;
                    path.pop();
                }
                cout << endl;
            }
        }

        while (!path.empty()) path.pop();
        if (!ans_metro[end].empty()) {
            info k = ans_metro[end][0];
            while (k.v != start) {
                if ((!path.empty()) && (k.v == end)) {
                    cout << "loop" << endl;
                    while (!path.empty()) path.pop();
                    break;
                }
                path.push(k.v);
                k = ans_metro[k.pre_v][k.pre_k];
            }
            if (!path.empty()) {
                real_path_num++;
                str += to_string(path.size()) + "\n";
                int now = start, next;
                while (!path.empty()) {
                    next = path.top();
                    cout << now << " -> " << next << " ";
                    str += to_string(now) + " " + to_string(next) + "\n";
                    now = next;
                    path.pop();
                }
            }

        }
        out << real_path_num << endl << str << endl;

    }

    return 0;
}


vector<vector<info>> ksp(const int s, const int MAX_NODE_ID, const int path_num, bool OUTPUT_FLAG, vector<edge> map_used[]){
    priority_queue<info> q;
    vector<vector<info> > dist(MAX_NODE_ID + 1);

    q.emplace(s, 0, -1, -1);
    while(!q.empty()){
        info e = q.top();
        q.pop();
        if (dist[e.v].size() >= path_num*5) continue;

        dist[e.v].push_back(e);

        for(auto i : map_used[e.v]){
            if (i.v != s) q.emplace(i.v, e.dis + i.dis, e.v, dist[e.v].size() - 1);
        }
    }

    if (OUTPUT_FLAG) {
        for (int i = 0; i <= MAX_NODE_ID; ++i) {
            cout << i << " : " << endl;
            stack<int> path;
            for (auto j: dist[i]) {
                info k = j;
                while (k.v != s) {
                    if ((!path.empty()) && (k.v == i)) {
                        cout << "loop" << endl;
                        while (!path.empty()) path.pop();
                        break;
                    }
                    path.push(k.v);
                    k = dist[k.pre_v][k.pre_k];
                }
                if (!path.empty()) {
                    cout << j.dis << ": " << s;
                    while (!path.empty()) {
                        cout << " -> " << path.top();
                        path.pop();
                    }
                    cout << endl;
                }
            }
            cout << endl;
        }
    }

    return dist;
}