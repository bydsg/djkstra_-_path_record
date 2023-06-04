#ifndef Graph_h
#define Graph_h
#include <iostream>
#include "vector"
using namespace std;
#define pi 3.14
#define maxSize  40

 struct Point {
    double x;
    double y;
    double l;
    double r;
    double s;
    double theta;
};
struct Road
{
    int id;
    std::vector<int> pre;
    std::vector<int> beh;
    double distance;
    std::vector<Point> road_points;

};
const int DefaultVertices = 30;
 
template <class T, class E>
struct Edge { // 边结点的定义
    int dest; // 边的另一顶点位置
    E cost; // 表上的权值
    Edge<T, E> *link; // 下一条边链指针
};
 
template <class T, class E>
struct Vertex { // 顶点的定义
    T data; // 顶点的名字
    Edge<T, E> *adj; // 边链表的头指针
};
 
template <class T, class E>
class Graphlnk {
public:
    const E maxValue = 100000; // 代表无穷大的值（=∞）
    Graphlnk(int sz=DefaultVertices); // 构造函数
    ~Graphlnk(); // 析构函数
    void inputGraph(std::vector<Road> roads); // 建立邻接表表示的图
    void outputGraph(); // 输出图中的所有顶点和边信息
    T getValue(int i); // 取位置为i的顶点中的值
    E getWeight(int v1, int v2); // 返回边（v1， v2）上的权值
    bool insertVertex(const T& vertex); // 插入顶点
    bool insertEdge(int v1, int v2, E weight); // 插入边
    bool removeVertex(int v); // 删除顶点
    bool removeEdge(int v1, int v2); // 删除边
    int getFirstNeighbor(int v); // 取顶点v的第一个邻接顶点
    int getNextNeighbor(int v,int w); // 取顶点v的邻接顶点w的下一邻接顶点
    int getVertexPos(const T vertex); // 给出顶点vertex在图中的位置
    int numberOfVertices(); // 当前顶点数
private:
    int maxVertices; // 图中最大的顶点数
    int numEdges; // 当前边数
    int numVertices; // 当前顶点数
    Vertex<T, E> * nodeTable; // 顶点表(各边链表的头结点)
};
 
// 构造函数:建立一个空的邻接表
template <class T, class E>
Graphlnk<T, E>::Graphlnk(int sz) {
    maxVertices = sz;
    numVertices = 0;
    numEdges = 0;
    nodeTable = new Vertex<T, E>[maxVertices]; // 创建顶点表数组
    if(nodeTable == NULL) {
        cerr << "存储空间分配错误！" << endl;
        exit(1);
    }
    for(int i = 0; i < maxVertices; i++)
        nodeTable[i].adj = NULL;
}
 
// 析构函数
template <class T, class E>
Graphlnk<T, E>::~Graphlnk() {
    // 删除各边链表中的结点
    for(int i = 0; i < numVertices; i++) {
        Edge<T, E> *p = nodeTable[i].adj; // 找到其对应链表的首结点
        while(p != NULL) { // 不断地删除第一个结点
            nodeTable[i].adj = p->link;
            delete p;
            p = nodeTable[i].adj;
        }
    }
    delete []nodeTable; // 删除顶点表数组
}
 
// 建立邻接表表示的图
template <class T, class E>
void Graphlnk<T, E>::inputGraph(std::vector<Road> roads) {
    int n, m; // 存储顶点树和边数
    int j1,k1;
    T e1, e2; // 顶点
    E weight; // 边的权值
    n =roads.size();//
    int sum =0;
    for (size_t i = 0; i < n; i++)
    {
        sum = sum+ roads[i].pre.size()+ roads[i].beh.size();
    }
    m = sum;

    for(size_t i = 0; i < n; i++) {
        insertVertex(roads[i].id); // 插入顶点
    }
    

 
   for (size_t j = 0; j < n; j++)
    {
        
        for (size_t i = 0; i < roads[j].pre.size(); i++)
        {
            e1 = roads[j].id;//2
            e2 = roads[j].pre[i];//1
            weight = 2;
            j1 = getVertexPos(e1);//1
            k1 = getVertexPos(e2);//0
            if(j1 == -1 || k1 == -1)
                cout << "边两端点信息有误，请重新输入！" << endl;
            else {
                insertEdge(j1, k1, weight); // 插入边
            }         
        }
        for (size_t i = 0; i < roads[j].beh.size(); i++)
        {   e1 = roads[j].id;
            e2 = roads[j].beh[i];
            weight = 2;
            j1 = getVertexPos(e1);
            k1 = getVertexPos(e2);
            if(j1 == -1 || k1 == -1)
                cout << "边两端点信息有误，请重新输入！" << endl;
            else {
                insertEdge(j1, k1, weight); // 插入边
            }
        }
    }

}
 
// 输出有向图中的所有顶点和边信息
template <class T, class E>
void Graphlnk<T, E>::outputGraph() {
    int n, m, i;
    T e1, e2; // 顶点
    E weight; // 权值
    Edge<T, E> *p;
    
    n = numVertices;
    m = numEdges;
    cout << "图中的顶点数为" << n << ",边数为" << m << endl;
    for(i = 0; i < n; i++) {
        p = nodeTable[i].adj;
        while(p != NULL) {
            e1 = getValue(i); // 有向边<i, p->dest>
            e2 = getValue(p->dest);
            weight = p->cost;
            cout << "<" << e1 << ", " << e2 << ", " << weight << ">" << endl;
            p = p->link; // 指向下一个邻接顶点
        }
    }
}
 
// 取位置为i的顶点中的值
template <class T, class E>
T Graphlnk<T, E>::getValue(int i) {
    if(i >= 0 && i < numVertices)
        return nodeTable[i].data;
    return NULL;
}
 
// 返回边（v1， v2）上的权值
template <class T, class E>
E Graphlnk<T, E>::getWeight(int v1, int v2) {
    if(v1 != -1 && v2 != -1) {
        Edge<T , E> *p = nodeTable[v1].adj; // v1的第一条关联的边
        while(p != NULL && p->dest != v2) { // 寻找邻接顶点v2
            p = p->link;
        }
        if(p != NULL)
            return p->cost;
    }
    return maxValue; // 边(v1, v2)不存在,就存放无穷大的值
}
 
// 插入顶点
template <class T, class E>
bool Graphlnk<T, E>::insertVertex(const T& vertex) {
    if(numVertices == maxVertices) // 顶点表满，不能插入
        return false;
    nodeTable[numVertices].data = vertex; // 插入在表的最后
    numVertices++;
    return true;
}
 
// 插入边
template <class T, class E>
bool Graphlnk<T, E>::insertEdge(int v1, int v2, E weight) {
    if(v1 >= 0 && v1 < numVertices && v2 >= 0 && v2 < numVertices) {
        Edge<T, E> *p = nodeTable[v1].adj; // v1对应的边链表头指针
        while(p != NULL && p->dest != v2) // 寻找邻接顶点v2
            p = p->link;
        if(p != NULL) // 已存在该边，不插入
            return false;

             
        p = new Edge<T, E>; // 创建新结点
        p->dest = v2;
        p->cost = weight;
        p->link = nodeTable[v1].adj; // 链入v1边链表
        nodeTable[v1].adj = p;
        numEdges++;
        return true;
    }
    return false;
}
 
// 有向图删除顶点较麻烦
template <class T, class E>
bool Graphlnk<T, E>::removeVertex(int v) {
    if(numVertices == 1 || v < 0 || v > numVertices)
        return false; // 表空或顶点号超出范围
    
    Edge<T, E> *p, *s;
    // 1.清除顶点v的边链表结点w 边<v,w>
    while(nodeTable[v].adj != NULL) {
        p = nodeTable[v].adj;
        nodeTable[v].adj = p->link;
        delete p;
        numEdges--; // 与顶点v相关联的边数减1
    } // while结束
    // 2.清除<w, v>，与v有关的边
    for(int i = 0; i < numVertices; i++) {
        if(i != v) { // 不是当前顶点v
            s = NULL;
            p = nodeTable[i].adj;
            while(p != NULL && p->dest != v) {// 在顶点i的链表中找v的顶点
                s = p;
                p = p->link; // 往后找
            }
            if(p != NULL) { // 找到了v的结点
                if(s == NULL) { // 说明p是nodeTable[i].adj
                    nodeTable[i].adj = p->link;
                } else {
                    s->link = p->link; // 保存p的下一个顶点信息
                }
                delete p; // 删除结点p
                numEdges--; // 与顶点v相关联的边数减1
            }
        }
    }
    numVertices--; // 图的顶点个数减1
    nodeTable[v].data = nodeTable[numVertices].data; // 填补,此时numVertices，比原来numVertices小1，所以，这里不需要numVertices-1
    nodeTable[v].adj = nodeTable[numVertices].adj;
    // 3.要将填补的顶点对应的位置改写
    for(int i = 0; i < numVertices; i++) {
        p = nodeTable[i].adj;
        while(p != NULL && p->dest != numVertices) // 在顶点i的链表中找numVertices的顶点
            p = p->link; // 往后找
        if(p != NULL) // 找到了numVertices的结点
            p->dest = v; // 将邻接顶点numVertices改成v
    }
    return true;
}
 
// 删除边
template <class T, class E>
bool Graphlnk<T, E>::removeEdge(int v1, int v2) {
    if(v1 != -1 && v2 != -1) {
        Edge<T, E> * p = nodeTable[v1].adj, *q = NULL;
        while(p != NULL && p->dest != v2) { // v1对应边链表中找被删除边
            q = p;
            p = p->link;
        }
        if(p != NULL) { // 找到被删除边结点
            if(q == NULL) // 删除的结点是边链表的首结点
                nodeTable[v1].adj = p->link;
            else
                q->link = p->link; // 不是，重新链接
            delete p;
            return true;
        }
    }
    return false; // 没有找到结点
}
 
// 取顶点v的第一个邻接顶点
template <class T, class E>
int Graphlnk<T, E>::getFirstNeighbor(int v) {
    if(v != -1) {
        Edge<T, E> *p = nodeTable[v].adj; // 对应链表第一个边结点
        if(p != NULL) // 存在，返回第一个邻接顶点
            return p->dest;
    }
    return -1; // 第一个邻接顶点不存在
}
 
// 取顶点v的邻接顶点w的下一邻接顶点
template <class T, class E>
int Graphlnk<T, E>::getNextNeighbor(int v,int w) {
    if(v != -1) {
        Edge<T, E> *p = nodeTable[v].adj; // 对应链表第一个边结点
        while(p != NULL && p->dest != w) // 寻找邻接顶点w
            p = p->link;
        if(p != NULL && p->link != NULL)
            return p->link->dest;  // 返回下一个邻接顶点
    }
    return -1; // 下一个邻接顶点不存在
}
 
// 给出顶点vertex在图中的位置
template <class T, class E>
int Graphlnk<T, E>::getVertexPos(const T vertex) {
    for(int i = 0; i < numVertices; i++)
        if(nodeTable[i].data == vertex)
            return i;
    return -1;
}
 
// 当前顶点数
template <class T, class E>
int Graphlnk<T, E>::numberOfVertices() {
    return numVertices;
}
 
#endif /* Graph_h */