#ifndef SRC_UTILITY_H_
#define SRC_UTILITY_H_

#include <vector>

using std::vector;


// Calculating Euclidean distance between two points
double distance(double x1, double y1, double x2, double y2);

// There is a map of all these waypoints around our highway
// And can see wich one is closest to us
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
					const vector<double> &maps_y);				

// Check others points it's like on the airplane where the nearest exist 
// might be behind you
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
				const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
// The Frenet Coordinates representing position on a road in a more 
// intuitive way than traditional (x,y)(x,y) Cartesian Coordinates
vector<double> getFrenet(double x, double y, double theta, 
						const vector<double> &maps_x, 
						const vector<double> &maps_y);


// Transform from Frenet s,d coordinates to Cartesian x,y
// Inverse from Frenet to Cartesian. It's not a linear transformation
// That something that's calculated at the very beginning that we can just feed it.
// And, that's used for the map inside the function itself
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

#endif

/* SRC_UTILITY_H_ */