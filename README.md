# Rapidly-exploring Random Tree <br><br>

### Brief overview
A rapidly-explore random tree (RRT) algorithm was developed by `Steven M. LaValle` and `James J. Kuffner Jr.` to search an environment effectively. A search tree is generated incrementally from a starting position and then randomly grows towards the unsearched area of the map. Three environmental conditions will be presented in this project.

### Results
* RRT with no obstacles (100 & 1000 iterations)
<img src="image/RRT0.gif" />
<img src="image/RRT1.gif" />
* RRT with circular obstacles
<img src="image/RRT2.gif" />
* RRT with arbitary obstacles
<img src="image/RRT3.gif" />

### Algorithm descriptions
* pseudocode for the basic algorithm (no obstacle)

<img src="image/RRT4.png" /><br>

* **RANDOM_CONFIGURATION** Generates a random position in the map
* **NEAREST_VERTEX** Finds the vertex in the G-list that is closest to the given position
* **NEW_CONFIGURATION** Generates a configuration by moving a distance from the nearest vertex towards the random position
* **CHECK_POINT_COLLISION** Checks if the point collides with the obstacles
* **CHECK_EDGE_COLLISION** Checks if the edge collides with the obstacles
* **CHECK_COLLISION_FREE_PATH** Check if there is a collision-free path to the goal position <br><br>

