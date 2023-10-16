# Shuttle bus design using col generation method
Code for shuttle bus service design and passenger flow assignment under UTR disruption using column generation method and gurobi optimizer. Model reproduced from *"Integrated optimization of bus bridging service design and passenger assignment in response to urban rail transit disruptions"* by Wang Yun et al. 2023. 

[DOI: 10.1016/j.trc.2023.104098](https://doi.org/10.1016/j.trc.2023.104098)

## 输入输出配置约定
**各个文件之间使用的结点编号要统一**

### Arc 配置文件
+ 读取接口：`Arcsets::readArcsFromFile(const string& Filename)`
+ 首行为一个整数 n，表示弧段总数
+ 接下来 n 行每行包含 6 个空格分割的输入，用于描述一个弧段，类型为`<int> <int> <int> <int> <double> <binary>`，表示弧段起点、终点、长度、最大运营频率、速度、弧段类型（0为地铁弧段、1为公交弧段）
+ 注意起点与终点**必须从 0 开始连续编号**
+ 同一 S-T 对要么是地铁弧要么是公交弧，不会同时存在

### Line 配置文件
+ 读取接口：`Linesets::readLinesFromFile(const string& Filename, Arcsets& Arcs)`
+ 首行为一个整数 n，表示线路总数
+ 接下来包含 n 个输入块
  + 第 i 个输入块首行为两个整数 `p<binary>` 和 `q<int>`，p 表示第 i 条线路的类型（0为地铁线路、1为公交线路），q 表示第 i 条线路包含的弧段数量
  + 接下来 q 行每行包含 2 个空格分割的输入，用于描述一条线路，类型为`<int> <int>`，表示弧段起点、终点

### 关于 KSP 问题的 OD 配置文件及输出
+ 首行为两个整数 n 和 k，表示 OD 对总数与每个 OD 对最大生成的路径数量
+ 接下来的 n 行每行包括三个整数，表示 OD 起点、终点、需求量
+ 输出一个 OD 配置文件，其格式符合下一小节的约定
+ **不是每个 OD 对都恰好包括 k 条路径，比如含环路径将被去除，部分给定的路网上亦无法得到 k 条路径**

### OD 配置文件
+ 该文件应该由上面的 KSP 问题生成
+ 读取接口：`ODsets::readODsFromFile(const string& Filename, Arcsets& Arcs)`
+ 首行为一个整数 n，表示 OD 对总数
+ 接下来的 n 个输入块每块描述一个 OD 及其路线
  + 块首行为 4 个空格分割的输入，类型为`<int> <int> <int> <int>`，表示 OD 起点、终点、需求量、路径数量
  + 接下来为路径数量个输入块，每块首行为一个整数 m，表示该路径包含的弧段数量
    + 接下来为 m 行，每行包含 2 个空格分割的输入，用于描述一条路径，类型为`<int> <int>`，表示弧段起点、终点
