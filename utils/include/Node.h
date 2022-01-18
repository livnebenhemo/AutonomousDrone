//
// Created by tzuk on 13/01/2022.
//

#ifndef ORB_SLAM2_NODE_H
#define ORB_SLAM2_NODE_H


#include <memory>
#include <utility>

class Node {
public:
    Node();
    Node(double x,double y);
    void SetCost(double cost){this->cost = cost;};
    void SetParent(std::shared_ptr<Node> node){parent = std::move(node);};
private:
    double x;
    double y;
    double cost;
    std::shared_ptr<Node> parent;
};


#endif //ORB_SLAM2_NODE_H
