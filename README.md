# RoboticsAlgorithms

### My Personal Space to practice Robotics Algorithms on planning, mapping, filtering & optimization.

### Will be updating all the time !

## Planning

- A* Planning\
  Simple A* planning with global obstacle map.
  ![](./results/planning/astar.png)
  Compare to Djikstra:
  ![](./results/planning/djikstra.png)

- PRM\
  Simple PRM planning with global obstacle map.
  ![PRM](./results/planning/prm.png)

- RRT\
  Simple RRT planning with global obstacle map.
  ![RRT](./results/planning/rrt.png)

  However, slow due to its limit in complex maps that has narrow paths.
  ![RRT2](./results/planning/rrt2.png)

  Can be updated to RRT*, which have optimized the path cost by rewiring.
  ![RRT*](./results/planning/rrt_star.png)

### Note: maps are from [an open source github repo](https://github.com/XM522706601/robotics_tutorial_for_zhihu)

## Optimization

- Minimum Snap Trajectory Optimization for quadrotor\
  TO DO: Add general version of implementation
  ![m_snap](./results/optimization/minimum_snapQP.png)
  >Reference: [1] Mellinger D, Kumar V. Minimum snap trajectory generation and control for quadrotors, Robotics and Automation (ICRA), 2011
