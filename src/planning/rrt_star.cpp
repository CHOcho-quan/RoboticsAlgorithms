//
// Created by quan
// 2020.3.21
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

class RRTStarPlanner {
    /**
     * Robotics RRT* Planner
     * @calcPath - calculating the final path and draw it on Obstacle map
     * @RRTStarPlanning - planning the path from start to end by RRT method
    */
 private:
    int expand_length;
    float towards_goal;
    float gamma_d;  // for finding near potential parents
 public:
    RRTStarPlanner(float t_g = 0.5, int e_l = 20, float gamma = 100) : expand_length(e_l), towards_goal(t_g), gamma_d(gamma) {}

    float randDirection() {
        /**
         * @brief - randomize a direction of rrt to expand
         * @return - return the theta angle of the direction
        */
        return rand() / double(RAND_MAX);
    }

    void calcPath(Node *goal, GlobalObstacleMap m) {
        /**
         * @brief - calculate the final path finded by RRT*
         * @param goal - goal place; m - obstacle map
        */
        while (goal != NULL) {
            Node *tmp = goal->parent;
            if (tmp != NULL) cv::line(m.background, cv::Point(goal->x, goal->y), cv::Point(tmp->x, tmp->y), cv::Scalar(128, 0, 128), 3);
            goal = goal->parent;
        }
    }

    Node* getNearestNode(vector<Node*> vec, cv::Point cp) {
        /**
         * @brief - get the nearest node inside the open list
         * @param vec - open list; cp - current point
         * @return - returning the nearest node
        */
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

    float distance(cv::Point p1, cv::Point p2) {
        /**
         * @brief - calculating the distance between point1 and point2
         * @param p1, p2 - the two points to be calculated distance
         * @return - returning the distance of the two points
        */
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }

    vector<Node*> nearSearch(Node* n, vector<Node*> l) {
        /**
         * @brief - RRT* searching near nodes to do rewiring
         * @param n - node to be searched as center; l - node list
         * @return - returning a vector containing all near nodes
        */
        vector<Node*> result;
        int nodes_num = l.size() + 1;
        float criterion = gamma_d * sqrt(log(nodes_num) / nodes_num);
        // cout << criterion << endl;
        for (int i = 0;i < l.size();i++) {
            if (distance(cv::Point(l[i]->x, l[i]->y), cv::Point(n->x, n->y)) < criterion) {
                result.push_back(l[i]);
            }
        }

        return result;
    }
    
    void RRTStarPlanning(GlobalObstacleMap m) {
        /**
         * @brief - RRT* planning algorithm
         * @param m - obstacle map
        */
        vector<Node *> waiting_list;

        Node *start, *final;
        start = new Node(m.start_x, m.start_y, 0.0);
        waiting_list.push_back(start);
        while (!waiting_list.empty())
        {
            // Do as normal RRT do
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

            Node *next, *tmp;
            int next_x = current->x + x_length * expand_length, next_y = current->y + y_length * expand_length;

            // Collision Check & End Status
            if (m.checkCell(next_x, next_y)) continue;
            if (m.heuristic(next_x, next_y) < expand_length) {
                cout << "Path Finded!" << endl;
                cv::line(m.background, cv::Point(current->x, current->y),
                                       cv::Point(m.goal_x, m.goal_y), cv::Scalar(0, 120, 0), 3);
                final = new Node(m.goal_x, m.goal_y, 0.0, current);
                break;
            }

            float next_dis = current->distance + distance(cv::Point(next_x, next_y), cv::Point(current->x, current->y));
            Node *true_parent = current;
            tmp = new Node(next_x, next_y, next_dis);
            // RRT* steps - adding parents refine
            if (DEBUG) cout << next_x << ' ' << next_y << endl;
            vector<Node*> potential_parents = nearSearch(tmp, waiting_list);
            for (int p = 0;p < potential_parents.size();p++) {
                Node *pp = potential_parents[p];
                if (!m.checkPath(cv::Point(pp->x, pp->y), cv::Point(next_x, next_y))) continue;
                float p_dis = pp->distance + distance(cv::Point(next_x, next_y), cv::Point(pp->x, pp->y));
                if (p_dis < next_dis) {
                    next_dis = p_dis;
                    true_parent = pp;
                }
            }

            //Draw the new node
            cv::line(m.background, cv::Point(true_parent->x, true_parent->y), 
                                   cv::Point(next_x, next_y), cv::Scalar(0, 120, 0), 3);
            cv::circle(m.background, cv::Point(next_x, next_y), 3, cv::Scalar(30, 50, 128), -1);

            // Rewire the whole tree
            next = new Node(next_x, next_y, next_dis, true_parent);
            waiting_list.push_back(next);
            for (int p = 0;p < potential_parents.size();p++) {
                Node *pp = potential_parents[p];
                if (pp->x == true_parent->x && pp->y == true_parent->y) continue;
                if (!m.checkPath(cv::Point(pp->x, pp->y), cv::Point(next_x, next_y))) continue;

                // If not this parent
                if (pp->distance > next_dis + distance(cv::Point(pp->x, pp->y), cv::Point(next->x, next->y))) {
                    // Now set pp's parent to next!
                    cv::line(m.background, cv::Point(pp->x, pp->y), 
                                   cv::Point(pp->parent->x, pp->parent->y), cv::Scalar(255, 255, 255), 3);
                    pp->parent = next;
                    cv::line(m.background, cv::Point(pp->x, pp->y), 
                                   cv::Point(next_x, next_y), cv::Scalar(0, 120, 0), 3);
                    if (DEBUG) cout << "Rewire!" << endl;
                }
            }

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
    RRTStarPlanner rrt_star = RRTStarPlanner();
    srand(int(time(NULL)));
    rrt_star.RRTStarPlanning(om);
    cv::resize(om.background, om.background, cv::Size(200, 200));
    cv::imwrite("../results/planning/rrt_star.png", om.background);
}