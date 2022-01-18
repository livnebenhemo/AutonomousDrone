//
// Created by tzuk on 13/01/2022.
//

#include "include/Node.h"

Node::Node() {
    x = 0.0;
    y = 0.0;
    cost = 0.0;
    parent = nullptr;
}

Node::Node(double x, double y) {
    this->x = x;
    this->y = y;
    cost = 0.0;
    parent = nullptr;
}