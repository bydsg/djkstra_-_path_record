#ifndef Dijkstra_h
#define Dijkstra_h
#include "Graph.h"

// Dijkstra算法
template <class T, class E>
void Dijkstra(Graphlnk<T, E> &G, int v, E dist[], int path[]) {
    // Graph是一个带权有向图，dist[]是当前求到的从顶点v到顶点j的最短路径长度，同时用数组
    // path[]存放求到的最短路径
    int n = G.numberOfVertices(); // 顶点数
    bool *s = new bool[n]; // 最短路径顶点集
    int i, j, k, u;
    E w, min;
    
    for(i = 0; i < n; i++) {
        dist[i] = G.getWeight(v,i); // 数组初始化，获取(v,i)边的权值
        s[i] = false; // 该顶点未被访问过
        if(i != v && dist[i] < G.maxValue) // 顶点i是v的邻接顶点
            path[i] = v; // 将v标记为顶点i的最短路径
        else
            path[i] = -1; // 说明该顶点i与顶点v没有边相连
    }
    s[v] = true; // 标记为访问过，顶点v加入s集合中
    dist[v] = 0;
    for(i = 0; i < n-1; i++) {
        min = G.maxValue;
        u = v; // 选不在生成树集合s[]中的顶点
        // 1.找v的权值最小且未被访问过的邻接顶点w,<v,w>
        for(j = 0; j < n; j++) {
            if(s[j] == false && dist[j] < min) {
                u = j;
                min = dist[j];
            }
        }
        s[u] = true; // 将顶点u加入到集合s
        for(k = 0; k < n; k++) { // 修改
            w = G.getWeight(u, k);
            if(s[k] == false && w < G.maxValue && dist[u] + w < dist[k]) {
                // 顶点k未被访问过，且从v->u->k的路径比v->k的路径短
                dist[k] = dist[u] + w;
                path[k] = u; // 修改到k的最短路径
            }
        }
    }
}
 
/*// 从path数组读取最短路径的算法
template <class T, class E>
void printShortestPath(Graphlnk<T, E> &G, int v, int e , E dist[], int path[]) {
    int i, j, k, n = G.numberOfVertices();
    int *d = new int[n];
    
    cout << "从顶点" << G.getValue(v) << "到终点的最短路径为: " << endl;
// for(i = 0; i < n; i++) 
        if(e != v) { // 如果不是顶点v
            j = e;
            k = 0;
            while(j != v) {
                d[k++] = j;
                j = path[j];
            }
            cout  << "顶点" << G.getValue(e) << "的最短路径为：" << G.getValue(v);
            while(k > 0)
                cout << "->" << G.getValue(d[--k]);
            cout << "，最短路径长度为：" << dist[e] << endl;
        }
   // }
}
#endif /* Dijkstra_h */



// 从path数组读取最短路径的算法
template <class T, class E>
std::vector<int>  printShortestPath(Graphlnk<T, E> &G, int v, int e , E dist[], int path[]) {
    std::vector<int> road_id;
    int i, j, k, n = G.numberOfVertices();
    int *d = new int[n];
    
    cout << "从道路" << G.getValue(v) << "到道路"<< G.getValue(e)<<"的最短路径为: " << endl;

        if(e != v) { // 如果不是顶点v
            j = e;
            k = 0;
            while(j != v) {
                d[k++] = j;
                j = path[j];
            }
            cout << G.getValue(v);
            road_id.push_back(G.getValue(v));
            while(k > 0){
                k--;
                cout << "->" << G.getValue(d[k]);
                road_id.push_back(G.getValue(d[k]));
            }  
        }
        if(e == v){
            cout << G.getValue(v);
            road_id.push_back(G.getValue(v));
        }
    return road_id;
}
#endif /* Dijkstra_h */