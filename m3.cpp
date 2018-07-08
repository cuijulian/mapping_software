#include "m1.h"
#include "m2.h"
#include "m3.h"

#include "Global.h"
#include "IntersectionNode.h"

#include <cmath>
#include <algorithm>
#include <vector>
#include <iterator>

#include <unordered_map>
#include <queue>
#include <iostream>
#include <iterator>     
#include <climits>

struct open_set_compare {

    bool operator()(const std::pair<double, unsigned>& lhs, const std::pair<double, unsigned>& rhs) const {
        return lhs.first > rhs.first;
    }
};

double
compute_path_travel_time(const std::vector<unsigned>& path,
        const double turn_penalty) {
    /**
     * 
     * @param path is given as a vector of street segment ids. This func can assume the vector forms a legal path or has size == 0.
     * @param turn_penalty
     * @return time required to travel along the path specified (seconds).
     * The travel time is the sum of the length/speed-limit of each street segment, 
     * plus the given turn_penalty (seconds) per turn implied by the path.
     * A turn occurs when 2 consecutive street segments have different street IDs.
     */
    unsigned path_size = path.size();
    double time = 0;

    if (path_size == 0) return 0;
    
    for (unsigned i = 0; i < path_size - 1; ++i) {
        time += travel_time[path[i]];

        if (getStreetSegmentInfo(path[i]).streetID != getStreetSegmentInfo(path[i + 1]).streetID)
            time += turn_penalty;
    }
    time += travel_time[path[path_size - 1]];

    return time;
}

double
heuristic_cost_estimate(unsigned intersect_id_1, unsigned intersect_id_2) {
 
    //   return 0;
    
    LatLon point1 = getIntersectionPosition(intersect_id_1);
    LatLon point2 = getIntersectionPosition(intersect_id_2);
    
    double point1_lat_rad = point1.lat() * DEG_TO_RAD;
    double point1_lon_rad = point1.lon() * DEG_TO_RAD;
    double point2_lat_rad = point2.lat() * DEG_TO_RAD;
    double point2_lon_rad = point2.lon() * DEG_TO_RAD;
    
    double latitudeAverage = (point1_lat_rad + point2_lat_rad)/2.0;
    
    double x1 = point1_lon_rad*cos(latitudeAverage);
    double x2 = point2_lon_rad*cos(latitudeAverage);
    
    double y1 = point1_lat_rad;
    double y2 = point2_lat_rad;

    // distance in meters
    double distance_squared = EARTH_RADIUS_IN_METERS * (pow((y2 - y1), 2) + pow((x2 - x1), 2));
    
    
    // get max_speed_limit (query r-tree)
    // estimate 150km/h = 150*1000m/3600s = 5*25m/3s (under-estimate time to travel)
   // return 3.0*distance/125.0; //returns time in sec

    // estimate 120km/h = 120*1000m/3600s = 100m/3s (under-estimate time to travel)
    return 3.0*distance_squared/100.0; //returns time in sec
     
}



// Prints shortest paths from src to all other vertices

std::vector<unsigned>
find_path_between_intersections(const unsigned intersect_id_start,
                                const unsigned intersect_id_end,
                                const double   turn_penalty) 
{
    if(intersect_id_start == intersect_id_end)
        return std::vector<unsigned>();
    /***********************************************************************************************************/
    /***********************************************************************************************************/
    /*** BASED ON: https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-priority_queue-stl/ ***/
    /***********************************************************************************************************/
    /***********************************************************************************************************/
    
    // priority queue to store nodes that are being preprocessed (openSet)
    std::priority_queue<std::pair<double, unsigned>, std::vector<std::pair<double, unsigned>>, open_set_compare> openSet;

    // closed set (set of already evaluated nodes)  
    std::vector<bool> closedSet(getNumberOfIntersections(), false);
    
    // "distance" vector contains times for each node 
    std::vector<double> dist(getNumberOfIntersections(), INT_MAX);

    // Insert starting node in priority queue and initialize its time to 0
    openSet.push(std::make_pair(0.0, intersect_id_start));
    dist[intersect_id_start] = 0.0;
    
    // best previous street seg
    std::vector<unsigned> optimal_prior_streetseg(getNumberOfIntersections(), 0);
    
    // best previous node
    std::vector<unsigned> best_prior_node(getNumberOfIntersections(), 0);

    // set of nodeID's (all keys from cameFromStreetSegment[])
    std::vector<bool> keys(getNumberOfIntersections(), false);
    
    /***********************************************************************************/
    /* Looping until priority queue becomes empty (or all distances are not finalized) */
    /***********************************************************************************/
    while( !openSet.empty() )
    {
        // Get top value (min distance) from set
        unsigned current = openSet.top().second;
        
        // if the current node == destination then break
        if(current == intersect_id_end)
            break; 
        
        // Pop it off the queue
        openSet.pop();
        
        // skip node if it is already in closed set
        if (closedSet[current])
            continue; 
        
        // get all adjacent nodes to current node       
        std::vector<IntersectionNode> neighbor_nodes = adjacencyList[current];
       
        if(current == intersect_id_start)
        {
            for (unsigned i = 0; i < neighbor_nodes.size(); ++i) 
            {
                // Get IntersectionNode object (nodeID, travelTImes, segmentIDs)
                IntersectionNode             n = neighbor_nodes[i];
                std::vector<double>    weights = n.travelTimes;
                std::vector<unsigned> segments = n.streetSegments;

                for (unsigned j = 0; j < weights.size(); ++j) 
                {
                    // If there is shorter path to n through u.
                    if (dist[n.nodeID] > (dist[current] + weights[j])) 
                    {
                        // [nodeID] -> [best Street Segment ID, best previous node]
                        optimal_prior_streetseg[n.nodeID] = segments[j];
                        best_prior_node[n.nodeID] = current;

                        // add to keys vector
                        keys[n.nodeID] = true;

                        // Updating distance of n
                        dist[n.nodeID] = dist[current] + weights[j];
                        openSet.push(std::make_pair(dist[n.nodeID], n.nodeID));
                    }
                }   
            }            
        }
        else
        {  
            for (unsigned i = 0; i < neighbor_nodes.size(); ++i) 
            {
                // Get IntersectionNode object (nodeID, travelTImes, segmentIDs)
                IntersectionNode             n = neighbor_nodes[i];
                std::vector<double>    weights = n.travelTimes;
                std::vector<unsigned> segments = n.streetSegments;

                unsigned previousSeg = optimal_prior_streetseg[current];

                for (unsigned j = 0; j < weights.size(); ++j) 
                {
                    // ADD TURN PENALTY TO WEIGHTS
                    if (street_segment_id_to_street_id[previousSeg] != street_segment_id_to_street_id[segments[j]])
                        weights[j] += turn_penalty;

                    // If there is shorter path to n through u.
                    if (dist[n.nodeID] > (dist[current] + weights[j])) 
                    {
                        // [nodeID] -> [best Street Segment ID, best previous node]
                        optimal_prior_streetseg[n.nodeID] = segments[j];
                        best_prior_node[n.nodeID] = current;

                        // add to keys vector
                        keys[n.nodeID] = true;

                        // Updating distance of n
                        dist[n.nodeID] = dist[current] + weights[j];
                        openSet.push(std::make_pair(dist[n.nodeID], n.nodeID));
                    }
                }
            }
        }
        // add current to the closedSet
        closedSet[current] = true;
    }
    
    /****************************************/
    /***** RECONSTRUCT THE SEGMENT PATH *****/
    /****************************************/

    unsigned key = intersect_id_end;

    std::vector<unsigned> totalSegments;    
    totalSegments.push_back(optimal_prior_streetseg[key]);
    
    while(keys[key]) 
    {
        key = best_prior_node[key];
        if (key != intersect_id_start) totalSegments.insert(totalSegments.begin(), optimal_prior_streetseg[key]);
    }
            
    if(key != intersect_id_start) return std::vector<unsigned>();
    
    return totalSegments;
}



std::vector<unsigned>
find_path_to_point_of_interest(const unsigned intersect_id_start,
                               const std::string point_of_interest_name,
                               const double turn_penalty) 
{
    // get all POI indexes corresponding to POI name
    std::vector<unsigned> POI_indexes = POI_name_to_POI_indexes[point_of_interest_name];
    std::vector<unsigned> POI_intersection_id;
    
    for (unsigned i = 0; i < POI_indexes.size(); i++) 
    {
        LatLon POI_location = getPointOfInterestPosition(POI_indexes[i]);
        unsigned closest_intersection = find_closest_intersection(POI_location);
        POI_intersection_id.push_back(closest_intersection);
    }

    unsigned intersection_end_id;

    // priority queue to store nodes that are being preprocessed (openSet)
    std::priority_queue<std::pair<double, unsigned>, std::vector<std::pair<double, unsigned>>, open_set_compare> openSet;

    // closed set (set of already evaluated nodes)
    std::vector<bool> closedSet(getNumberOfIntersections(), false);

    // "distance" vector contains times for each node 
    std::vector<double> dist(getNumberOfIntersections(), INT_MAX);
    
    bool found = false;

    // Insert starting node in priority queue and initialize its time to 0
    openSet.push(std::make_pair(0, intersect_id_start));
    dist[intersect_id_start] = 0;

    // cameFromStreetSegment[nodeID] -> [optimal prior Street Segment ID, best previous node]
    std::unordered_map<unsigned, std::pair<unsigned, unsigned>> cameFromStreetSegment;

    /***********************************************************************************************************************/
    /* Looping until reaching the intersection of POI or priority queue becomes empty (or all distances are not finalized) */
    /***********************************************************************************************************************/
    while (!openSet.empty() && !found) 
    {
        // Get top value (min distance) from set
        unsigned current = openSet.top().second;
        
        //reaching the intersection of POI
        for (unsigned i = 0; i < POI_intersection_id.size(); i++) 
        {
            if(current==POI_intersection_id[i])
            {
                found=true;
                intersection_end_id=POI_intersection_id[i];
                break;
            }
        }
        
        if (found) break;
        
        // Pop it off the queue
        openSet.pop();

        // skip node if it is already in closed set
        if (closedSet[current])
            continue;
        
        // get all adjacent nodes to current node       
        std::vector<IntersectionNode> neighbor_nodes = adjacencyList[current];

        // loop through all neighbors of current
        for (unsigned i = 0; i < neighbor_nodes.size(); ++i) 
        {
            // Get IntersectionNode object (nodeID, travelTImes, segmentIDs)
            IntersectionNode n = neighbor_nodes[i];
            std::vector<double> weights = n.travelTimes;
            std::vector<unsigned> segments = n.streetSegments;

            if (current != intersect_id_start) {
                unsigned previousSeg = cameFromStreetSegment[current].first;

                // ADD TURN PENALTY TO WEIGHTS
                for (unsigned j = 0; j < weights.size(); ++j)
                    if (getStreetSegmentInfo(previousSeg).streetID != getStreetSegmentInfo(segments[j]).streetID)
                        weights[j] += turn_penalty;
            }

            for (unsigned j = 0; j < weights.size(); ++j) {
                //  If there is shorter path to n through u.
                if (dist[n.nodeID] > (dist[current] + weights[j])) {
                    // [nodeID] -> [best Street Segment ID, best previous node]
                    cameFromStreetSegment[n.nodeID].first = segments[j];
                    cameFromStreetSegment[n.nodeID].second = current;

                    // Updating distance of n
                    dist[n.nodeID] = dist[current] + weights[j];
                    openSet.push(std::make_pair(dist[n.nodeID], n.nodeID));
                }
            }
        }
        // add current to the closedSet
        closedSet[current] = true;
    }

    /*************************************/
    /***** RECONSTRUCT THE NODE PATH *****/
    /*************************************/

    // get all keys of cameFromStreetSegment
    std::vector<unsigned> keys;
    keys.reserve(cameFromStreetSegment.size());

    for (auto kv : cameFromStreetSegment)
        keys.push_back(kv.first);

    // reconstruct node (intersection) path
    std::vector<unsigned> totalPath;
    unsigned key = intersection_end_id;
    totalPath.push_back(key);

    while (std::find(keys.begin(), keys.end(), key) != keys.end()) {
        key = cameFromStreetSegment[key].second;
        totalPath.push_back(key);
    }

    /****************************************/
    /***** RECONSTRUCT THE SEGMENT PATH *****/
    /****************************************/
    
    unsigned total_path_size = totalPath.size();
    std::vector<unsigned> totalSegments;

    for(int i = total_path_size-2; i >= 0; --i)  
        totalSegments.push_back(cameFromStreetSegment[totalPath[i]].first);
    
    return totalSegments;
}