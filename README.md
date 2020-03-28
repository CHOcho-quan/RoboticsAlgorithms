# RoboticsAlgorithms

### My Personal Space to practice Robotics Algorithms on planning, mapping, filtering & optimization.

### Will be updating all the time !

## Planning

- Search-based: A* Planning\
  Simple A* planning with global obstacle map, compared to djikstra.\
  !["A*"](./results/planning/astar.png) !["Djikstra"](./results/planning/djikstra.png)\

- Sample-based: PRM & RRT\
  Simple PRM & RRT(RRT*) planning with global obstacle map.\
  !["A*"](./results/planning/prm.png) !["Djikstra"](./results/planning/rrt.png)\
  RRT* can get the result optimized, but they both perform bad in maps having narrow channels.\
  !["A*"](./results/planning/rrt_star.png) !["Djikstra"](./results/planning/rrt2.png)

### Note: maps are from an open source github  [repo](https://github.com/XM522706601/robotics_tutorial_for_zhihu)

## Optimization

- Minimum Snap Trajectory Optimization for quadrotor\
  TO DO: Add general version of implementation
  ![Minimum_snap](./results/optimization/minimum_snapQP.png)
  >Reference: [1] Mellinger D, Kumar V. Minimum snap trajectory generation and control for quadrotors, Robotics and Automation (ICRA), 2011
