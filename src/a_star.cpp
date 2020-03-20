//
// Created by quan
// 2020.3.15
//
#include "a_star.h"

#define DEBUG 0

struct Node {
    /**
     * Node for A* planning
     * @x, y - the coordinate of current point
     * @cost - the f cost of the point computed by A*
    */
    int x;
    int y;
    float cost;
    Node *pre;
    Node(int x_c, int y_c, float c, Node *p=NULL) : x(x_c), y(y_c), cost(c), pre(p) {}
};

class AstarPlanner {
    /**
     * Robotics A* Planner
     * @calcPath - calculating the final path and draw it on Obstacle map
     * @AstarPlanning - plan the path on the given obstacle map
    */
 private:
    vector<vector<int> > close_list;
    vector<vector<float> > path_cost;
    vector<Node> motions = { Node(1, 0, 1.0), Node(0, 1, 1.0), Node(0, -1, 1.0), Node(-1, 0, 1.0), 
                            Node(1, 1, sqrt(2)), Node(1, -1, sqrt(2)), Node(-1, -1, sqrt(2)), Node(-1, 1, sqrt(2))};

 public:
    void calcPath(Node *end, GlobalObstacleMap om)
    {
        while (end != NULL)
        {
            om.annoteCell(end->x, end->y, PATH);
            end = end->pre;
        }
    }

    void AstarPlanning(GlobalObstacleMap om)
    {
        // Initialize open & close lists
        for (int i = 0;i < om.map_size_x;i++) {
            vector<int> tmp;
            vector<float> tmp_c;
            for (int j = 0;j < om.map_size_y;j++) {
                tmp.push_back(0);
                tmp_c.push_back(2147483647);
            }
            path_cost.push_back(tmp_c);
            close_list.push_back(tmp);
        }

        auto cmp = [](const Node *a, const Node *b) { return a->cost > b->cost; };
        priority_queue<Node*, vector<Node*>, decltype(cmp)> open(cmp);

        Node *start;
        start = new Node(om.start_x, om.start_y, 0.0);
        path_cost[om.start_x][om.start_y] = 0.0;
        open.push(start);

        bool ending_flag = false;
        Node *final;
        while (!open.empty())
        {
            // Smallest element in open list, put it into close list
            Node *current = open.top();
            open.pop();
            if (close_list[current->x][current->y]) continue;
            if (DEBUG) cout << current->cost << endl;
            close_list[current->x][current->y] = 1;

            om.annoteCell(current->x, current->y, ROBOT);
            om.render(0);

            // For every motion, get its next Node
            if (DEBUG) cout << "-----------next----------" << endl;
            for (auto motion : motions)
            {
                int next_x = current->x + motion.x, next_y = current->y + motion.y;
                float current_path_cost = path_cost[current->x][current->y];

                Node *next;
                if (om.checkCell(next_x, next_y) == GOAL) {
                    next = new Node(next_x, next_y, 0.0, current);
                    final = next;
                    ending_flag = true;
                    break;
                }
                if (close_list[next_x][next_y] || om.checkCell(next_x, next_y) == OBSTACLE) continue;

                float next_cost = current_path_cost + om.heuristic(next_x, next_y) + motion.cost;
                next = new Node(next_x, next_y, next_cost, current);
                path_cost[next_x][next_y] = min(path_cost[next_x][next_y], current_path_cost + motion.cost);
                open.push(next);
                if (DEBUG) cout << next->cost << endl;
            }
            if (DEBUG) cout << "-----------next----------" << endl;

            if (ending_flag) break;
        }

        calcPath(final, om);
        om.render(0);
    }
};

int main()
{
    vector<int> o_x, o_y;
    for (int i = 0;i < 50;i++) {
        o_x.push_back(0);
        o_y.push_back(i);
    }
    for (int i = 0;i < 50;i++) {
        o_x.push_back(i);
        o_y.push_back(0);
    }
    for (int i = 0;i < 50;i++) {
        o_x.push_back(i);
        o_y.push_back(49);
    }
    for (int i = 0;i < 50;i++) {
        o_x.push_back(49);
        o_y.push_back(i);
    }
    for (int i = 0;i < 26;i++) {
        o_x.push_back(i);
        o_y.push_back(15);
    }
    for (int i = 0;i < 26;i++) {
        o_x.push_back(50 - i);
        o_y.push_back(35);
    }
    GlobalObstacleMap m(50, 50, 5, 5, 45, 45, o_x, o_y);
    AstarPlanner planner;
    planner.AstarPlanning(m);
    m.render(0);
}