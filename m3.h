#pragma once

#include <vector>
#include <string>


// Returns the time required to travel along the path specified, in seconds.
// The path is given as a vector of street segment ids, and this function
// can assume the vector either forms a legal path or has size == 0.
// The travel time is the sum of the length/speed-limit of each street
// segment, plus the given turn_penalty (in seconds) per turn implied by the path.
// A turn occurs when two consecutive street segments have different street IDs.
double compute_path_travel_time(const std::vector<unsigned>& path,
                                const double turn_penalty);


// Returns a path (route) between the start intersection and the end
// intersection, if one exists. This routine should return the shortest path
// between the given intersections when the time penalty to turn (change
// street IDs) is given by turn_penalty (in seconds).
// If no path exists, this routine returns an empty (size == 0) vector.
// If more than one path exists, the path with the shortest travel time is
// returned. The path is returned as a vector of street segment ids; traversing
// these street segments, in the returned order, would take one from the start
// to the end intersection.
std::vector<unsigned> find_path_between_intersections(const unsigned intersect_id_start,
                                                      const unsigned intersect_id_end,
                                                      const double turn_penalty);


// Returns the shortest travel time path (vector of street segments) from
// the start intersection to a point of interest with the specified name.
// The path will begin at the specified intersection, and end on the
// intersection that is closest (in Euclidean distance) to the point of
// interest.
// If no such path exists, returns an empty (size == 0) vector.
std::vector<unsigned> find_path_to_point_of_interest(const unsigned intersect_id_start,
                                                     const std::string point_of_interest_name,
                                                     const double turn_penalty);