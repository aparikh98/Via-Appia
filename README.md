# Via-Appia
Pathfinding algorithm using D* lite with limited scope
Created by Kevin Wang, Justin Lin, Aakash Parikh

There are many different kinds of maze solving algorithms. Our implementation of D*Lite enables the scope feature to analyze obstacles within sight.
Building on A* and LPA*, D* lite uses a dynamically enabled algorithm with heuristic weighting. We built a modified D*lite algorithm, to dynamically change based on surroundings. Given a starting point and a goal, you will move closer to it, and react to obstacles that come into view. Our algorithm limits the range of vision of the moving object, and only calculates a path based on obstacles that it can see or has seen previously. In today's world, the increasing usage of automation requires self guided movement without the interference of humans. For example, in industrial assembly lines, six axis robotic arms use algorithm determined motion planning. Additionally, the Mars rovers developed by NASA also use an implementation of D* for their movement across the red planet's surface. Also, a change to be soon happening in the average human's life will be the introduction of autonomous vehicles which also use dynamic pathfinding to plan movement in terrain where complete information is not readily available. Dynamic pathing is critical in these situations to reach the destination.


Devpost link: https://devpost.com/software/pathfinding-6noj4
