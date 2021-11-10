

// Remember to check whether a point is inside the map, especially while checking for the collision with the obstacles.
// Sometimes the points outside the map could still give a feasible value (as it gives the value at other side of the map.)
// Convention of map in the codes; the map should be a grayscale with unsigned 8 bit int type.
// Begin classes with Upper Case letters, lower case for functions and "_" for naming the variables.
// Typedef the variable that are used more often, such as 'float' in std::pair for points and 'int' for indices of matrices.
// Using dictionary or vector takes time.
// For n in global planner (n = 30), use time_jump = math.sqrt(2)/velocity.

// Make seperate functions for ellipse and circle or line and make sure to
// check the given inputs are fesasible or not, as writing the direct functions in the main functions may throw some errors.

// Once check the transformation matrices.
// Change the Heuristic back to grassfire in python and cpp.
// Remember to consider the width of the vehicle while obstacle checking.
// Use return by reference for returning huge data.
// The previous iterations before executing the Pure Pursuit causing the path to move far away from obstacles and when the pure
// pursuit is running the resultant path is wierd, solve the issue.