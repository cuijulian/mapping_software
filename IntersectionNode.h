#pragma once

#include <vector>

class IntersectionNode {
public:
    unsigned nodeID;                      // ID of the intersection
    std::vector<unsigned> streetSegments; // streetSegments: 2-way or going to nodeID
    std::vector<double> travelTimes;      // travel time of each street segment
    IntersectionNode();
    IntersectionNode(unsigned nodeID_, std::vector<unsigned> streetSegments_, std::vector<double> travelTimes_); 
};