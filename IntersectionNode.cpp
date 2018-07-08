#include "IntersectionNode.h"


IntersectionNode::IntersectionNode()
{
    nodeID = 0;
    streetSegments = std::vector<unsigned>();       
    travelTimes = std::vector<double>();       
}


IntersectionNode::IntersectionNode(unsigned nodeID_, std::vector<unsigned> streetSegments_, std::vector<double> travelTimes_)
{
    nodeID = nodeID_;
    streetSegments = streetSegments_;      
    travelTimes = travelTimes_; 
}
