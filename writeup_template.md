## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`


**motion_planning.py**

This file differs from `backyard_flyer_solution` in that the `States` enum has an additional state:  `PLANNING`.  The planning state takes into account potential obstacles and plans a path to reach it's goal location.

In the `plan_path` method which is called once the drone is armed, we enter the `PLANNING` state, then a grid of obstacles is read from our CSV file, and a goal of 10m NE from our start position is set before running A\* to get to the goal.

Planning occurs in the `plan_path` method as follows:

1. Reads obstacle data from a collider file
2. Creates a 2D grid of the configuration space with the imported `create_grid` method
3. Defines the start and goal points
4. Using the `a-star` method, an A* search is implemented to find a path from start to goal.
5. The planned path is converted into waypoints


**planning_utils.py**

`create_grid` and `a_star` are the main utilities here. `a_star` includes an `Actions` classs and `valid_actions` method to help traverse the grid. The zig-zag path towards the goal is due to the UP/DOWN/LEFT/RIGHT action directions the drone is limited to.

**States**

The different states and their order of implementation:

| State     | Purpose                    | Order |
| --------- | -------------------------- | ----- |
| MANUAL    | Manual flight        | 1     |
| ARMING    | Start the drone propellers | 2     |
| PLANNING  | Plan a path given a map of obstacles     | 3     |
| TAKEOFF   | Ascend to a given altitude  | 4     |
| WAYPOINT  | Move between waypoints   | 5     |
| LANDING   | Reached goal, vertical descent   | 6     |
| DISARMING | Turn off propellers          | 7     |


### Implementing Your Path Planning Algorithm

#### 1. Set your global home position

These are the first lines of the `colliders.csv`:

```c
lat0 37.792480, lon0 -122.397450
posX,posY,posZ,halfSizeX,halfSizeY,halfSizeZ
```

These coordinates were extracted and set to home position and all movement would be measured from that location:

```py
with open('colliders.csv') as f:
            first_line = f.readline().strip()
        latlon = first_line.split(',')
        lon0 = float(latlon[0].strip().split(' ')[1])
        lat0 = float(latlon[1].strip().split(' ')[1])
		self.set_home_position(lat0, lon0, 0)        
        
```

#### 2. Set your current local position
The `global_to_local` function converts latititude / longitude to grid coordinates.

```py
local_pos = global_to_local(self.global_position,
                                    global_home=self.global_home)
```

#### 3. Set grid start position from local position
Calculating the local position helps in determiming where we are on the grid (for calculating A\*):

```py
grid_start = (int(np.rint(north - north_offset)),
                      int(np.rint(east - east_offset)))
```

#### 4. Set grid goal position from geodetic coords

There are two ways to set the grid goal:

1. Send geodetic coordinates(lat, lon) 
2. If there is no parameter sent, then the plan_path method sets the goal randomly.

The method converts the goal location to local coordinates, and then the grid coordinates. If it exceeds an edge of the grid, then the value is changed back to the value of the edge in order to stay in the grid.

For 3D planning, the randomly assigned altitude value has a maximum that is defined by the MAX_ALTITUDE.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)

For 2D planning, the `Action` object in the `planning_utils.py` file includes diagonal movement.

In the first version of the `a_star` function in the `planning_utils.py`, when a node can be reached with a lower cost after it was added to the branch, it is not possible to change the path for that node. In order to correct this error, the code location to add a node to the visited list is changed. A node is added to the visited list, only when it is the current node. Previously, it was added when it was the next_node. This change enabled a node to be placed in the queue more than once. It will be visited with the lowest queue cost. After it is visited, the other items in the queue, containing the same node, will be skipped without processing.

One problem was timeout. If planning takes too much time, it can't send the waypoints to the simulator. Increasing the timeout parameter of the connection object was not a solution. In that case, the MotionPlanning (Drone) instance and the simulator stuck at the TAKEOFF state.

The solution for this problem, creating a new MotionPlanning instance and sending the waypoints with that instance.

Another problem was zigzag movement. The solution was to first check the previous action. For a new node, if applying the previous action is valid, then first extend to that node. This provide a more prioritized order to that action in the queue.

Another thing to try was adding a cost for changing action. This cost shall be significantly lower than the action cost (action cost / 100). Otherwise, it changes the behavior of the a_star planning. This cost increased the process time by 50%. Therefore, it doesn't present in the current planner.

To increase process speed, number of steps taken with each step was increased. Current planner takes five steps with a single action. If the current node and the goal are closer than 5 * sqrt(2), then it is only possible to move a single step. The parameter to modify the number of steps is `maxmove` in the `planpath` method.

#### 6. Cull waypoints 


The `collinearity_check` was performed to determine whether a point in the path can be removed or not. If three points fit in the same line, then the second point is removed.

### 3D Grid A* algorithm
The 3D A* planning is implemented in the motionplot3Dvox notebook.

There are two different voxel maps. The former, which is the smaller one, has a voxel size of 5, and the latter has 1 voxel size. The main planning runs on the smaller voxel map. Since the voxel representation of the start and the goal points may not correspond to the actual points, the other planning calculates the path from start point to voxel and the path from voxel to goal point on the larger voxel map with voxel size 1. 

To decrease the planning time, the a_star algorithm runs in both ways. From start to goal and from goal to start. In cases such as planning to a hole, this algorithm significantly lowers the computation time. When a node is visited by both of the paths, it is accepted as the midpoint. And the path from start to the midpoint and the path from the midpoint to the goal are combined to represent the complete path.

### Graph A* algorithm
The 2D Graph A* planning is implemented in the motionplotgraph notebook. 2D grid A* planning converted to plan from a graph instead of a grid. There is Grid class to store created grid and graph instances.


### Probabilistic Roadmap
The Probabilistic Roadmap is implemented in the probabilisticroadmap notebook. 2D Graph A* planning converted to create a graph with a probabilistic method. There is GridProbabilistic class to store created grid and graph instances, and the hyper parameters and methods to create a probabilistic graph.



### RRT
The Rapidly-Exploring Random Tree is implemented in the rrt notebook. The probabilistic Roadmap converted to the RRT method. There is GridRRT class to store created grid and graph instances, and the hyper parameters and methods to create a rrt.


### Execute the flight
#### 1. Does it work?
It works!




