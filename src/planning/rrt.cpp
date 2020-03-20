//
// Created by quan
// 2020.3.18
//
#include "rrt.h"

struct Node {
    /**
     * Node for RRT planning
     * @x, y - the coordinate of current point
     * @distance - the distance of the point to goal computed by heuristic
    */
    int x;
    int y;
    float distance;
    Node *parent;
    Node(int x_c, int y_c, float c, Node *p=NULL) : x(x_c), y(y_c), distance(c), parent(p) {}
};

class RRTPlanner {
    /**
     * Robotics RRT Planner
     * @calcPath - calculating the final path and draw it on Obstacle map
     * @RRTPlanning - planning the path from start to end by RRT method
    */
 private:
    int expand_length;
    float towards_goal;
 public:
    RRTPlanner(float t_g = 0.5, int e_l = 20) : expand_length(e_l), towards_goal(t_g) {}

    float randDirection() {
        return rand() / double(RAND_MAX);
    }

    void calcPath(Node *goal, GlobalObstacleMap m) {
        while (goal != NULL) {
            Node *tmp = goal->parent;
            if (tmp != NULL) cv::line(m.background, cv::Point(goal->x, goal->y), cv::Point(tmp->x, tmp->y), cv::Scalar(128, 0, 128), 3);
            goal = goal->parent;
        }
    }

    Node* getNearestNode(vector<Node*> vec, cv::Point cp) {
        float min_distance = 2147483647;
        Node *re;
        for (int i = 0;i < vec.size();i++) {
            float dis = sqrt(pow(cp.x - vec[i]->x, 2) + pow(cp.y - vec[i]->y, 2));
            if (dis < min_distance) {
                min_distance = dis;
                re = vec[i];
            }
        }

        return re;
    }
    
    void RRTPlanning(GlobalObstacleMap m) {
        vector<Node *> waiting_list;

        Node *start, *final;
        start = new Node(m.start_x, m.start_y, 2147483647);
        waiting_list.push_back(start);
        while (!waiting_list.empty())
        {
            float p = randDirection();
            cv::Point target;
            
            if (p > towards_goal) {
                int t_x = rand() % m.map_size_x;
                int t_y = rand() % m.map_size_y;

                target = cv::Point(t_x, t_y);// random select
            }
            else target = cv::Point(m.goal_x, m.goal_y); // towards goal

            Node *current = getNearestNode(waiting_list, target);
            if (DEBUG) cout << current->x << ' ' << current->y << endl;

            // Towards target, move expand_length distance!
            float total = sqrt(pow(target.x - current->x, 2) + pow(target.y - current->y, 2));
            float x_length = (target.x - current->x) / total;
            float y_length = (target.y - current->y) / total;

            Node *next;
            int next_x = current->x + x_length * expand_length, next_y = current->y + y_length * expand_length;

            // Collision Check
            if (m.checkCell(next_x, next_y)) continue;
            if (m.heuristic(next_x, next_y) < expand_length) {
                cout << "Path Finded!" << endl;
                cv::line(m.background, cv::Point(current->x, current->y),
                                       cv::Point(m.goal_x, m.goal_y), cv::Scalar(0, 120, 0), 3);
                final = new Node(m.goal_x, m.goal_y, 0.0, current);
                break;
            }

            next = new Node(next_x, next_y, m.heuristic(next_x, next_y), current);

            waiting_list.push_back(next);

            //Draw the new node
            cv::line(m.background, cv::Point(current->x, current->y), 
                                   cv::Point(next_x, next_y), cv::Scalar(0, 120, 0), 3);
            cv::circle(m.background, cv::Point(next_x, next_y), 3, cv::Scalar(30, 50, 128), -1);
            m.render(0);
        }

        calcPath(final, m);
        m.render(0);
    }
};

int main()
{
    GlobalObstacleMap om(10, 10, 450, 450);
    om.render(0);
    RRTPlanner rrt = RRTPlanner();
    srand(int(time(NULL)));
    rrt.RRTPlanning(om);
    cout << rrt.randDirection() << endl;
}