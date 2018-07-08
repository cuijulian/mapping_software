#include "Global.h"
#include "m1.h"
#include <math.h>
#include <algorithm>


/*****************************************/
/***** DEFINE GLOBAL DATA STRUCTURES *****/
/*****************************************/
std::unordered_map<std::string, std::vector<unsigned>> street_name_to_id_map;               // streetName      -> streetIDs
std::unordered_map<unsigned, std::vector<unsigned>> street_id_to_street_segments_map;       // streetID        -> <streetSegments>
std::unordered_map<unsigned, std::vector<unsigned>> intersection_id_to_street_segments;     // intersectionID  -> <streetSegments>
std::unordered_map<unsigned, std::vector<LatLon>> street_segment_id_to_curve_points;        // streetSegmentID -> <LatLon points>
std::vector <LatLon> intersection_id_to_position;                                           // intersectionID -> <LatLon points>
std::unordered_map<unsigned, std::vector<unsigned>> street_ids_to_intersection_ids;         // streetID        -> <intersectionIDs>
std::vector <unsigned>  street_segment_id_to_street_id;                                     // streetSegmentID -> streetID
std::unordered_map<unsigned, std::vector<double>> street_segment_id_to_curve_point_lengths; // streetSegmentID -> <distances>
std::vector <std::string> street_segment_id_to_street_name;                                 // streetSegmentID -> streetName
std::vector <double> travel_time;                                                           // streetSegmentID -> travel time
std::unordered_map <std::string, std::vector<unsigned>> POI_name_to_POI_indexes;            // POI_name        -> POI_index(s)
rtree IntersectionTree;                                                                     // rtree for intersections
rtree PositionTree;                                                                         // rtree for POI
std::vector<std::vector<IntersectionNode>> adjacencyList;


double average_lat = 0;

/***********************************/
/***** DEFINE USEFUL FUNCTIONS *****/
/***********************************/

//Converts a LatLon point into Cartesian system.
//Returns a vector of x, y, and z coords.
std::vector<double>
to_cartesian_3D(LatLon point_) // converter between LatLon and Cartesian system (x, y, z)
{
    double point_lat_rad = point_.lat() * DEG_TO_RAD;
    double point_lon_rad = point_.lon() * DEG_TO_RAD;
    
    double x = cos(point_lon_rad) * cos(point_lat_rad);
    double y = sin(point_lon_rad) * cos(point_lat_rad);
    double z = sin(point_lat_rad);
    
    std::vector<double> cartesian_point = {x, y, z};
    
    return cartesian_point;
}

std::vector<unsigned>
street_segments_in_common(const unsigned intersect_id_1,
                          const unsigned intersect_id_2)
{
    std::vector<unsigned> street_seg_start = find_intersection_street_segments(intersect_id_1);
    std::vector<unsigned> street_seg_dest  = find_intersection_street_segments(intersect_id_2);
    std::vector<unsigned> result = std::vector<unsigned>();
    
    for(unsigned i = 0; i < street_seg_start.size(); ++i)
        for(unsigned j = 0; j < street_seg_dest.size(); ++j)
            if(street_seg_start[i] == street_seg_dest[j]) 
                result.push_back(street_seg_start[i]);
    
    return result;
}

std::vector<double> 
find_time_between_intersections(const unsigned intersect_id_start,
                                const unsigned intersect_id_end)
{    
    std::vector<unsigned> common = street_segments_in_common(intersect_id_start, intersect_id_end);
    std::vector<double> result = std::vector<double>();;
            
    for(unsigned i = 0; i < common.size(); ++i)      
        
        result.push_back(travel_time[common[i]]);
            
    return result;
}

/*************************************************/
/***** FUNCTION FILLS GLOBAL DATA STRUCTURES *****/
/*************************************************/
void 
initialize_global_database()
{
    unsigned num_of_streets         = getNumberOfStreets();
    unsigned num_of_street_segments = getNumberOfStreetSegments();
    unsigned num_of_intersections   = getNumberOfIntersections();
    //----------------------------------------//
    
    //----------------------------------------//
    //----- SETUP: street_name_to_id_map -----//
    //----------------------------------------//

    //Loops through all streets and stores id
    for (unsigned i = 0; i < num_of_streets; ++i)
        street_name_to_id_map[getStreetName(i)].push_back(i);

    //---------------------------------------------------//
    //----- SETUP: street_id_to_street_segments_map -----//
    //---------------------------------------------------//
    
    //Loops through all street segments and stores its street id
    for (unsigned i = 0; i < num_of_street_segments; ++i)
        street_id_to_street_segments_map[getStreetSegmentInfo(i).streetID].push_back(i);

    //-----------------------------------------------------//
    //----- SETUP: intersection_id_to_street_segments -----//
    //-----------------------------------------------------//

    //Loops through all intersections and stores its street segment id
    for (unsigned i = 0; i < num_of_intersections; ++i) {
        unsigned num_of_segments = getIntersectionStreetSegmentCount(i);

        for (unsigned j = 0; j < num_of_segments; ++j)
            intersection_id_to_street_segments[i].push_back(getIntersectionStreetSegment(i, j));
    }

    //----------------------------------------------------//
    //----- SETUP: street_segment_id_to_curve_points -----//
    //----------------------------------------------------//

    //Loops through all street segments and stores the LatLon of its curve points
    for (unsigned i = 0; i < num_of_street_segments; ++i) {
        if (getStreetSegmentInfo(i).curvePointCount == 0)
            street_segment_id_to_curve_points[i] = std::vector<LatLon>();
        else
            for (unsigned j = 0; j < getStreetSegmentInfo(i).curvePointCount; ++j)
                street_segment_id_to_curve_points[i].push_back(getStreetSegmentCurvePoint(i, j));
    }

    
    //----------------------------------------------------//
    //----- SETUP: intersection_id_to_position -----//
    //----------------------------------------------------//
     for (unsigned i = 0; i < num_of_intersections; ++i) {
         LatLon point = getIntersectionPosition(i);
       //  intersection_id_to_position.push_back(point);
         intersection_id_to_position.push_back(point);
     }
    
    
    
    
    
    //-------------------------------------------------//
    //----- SETUP: street_ids_to_intersection_ids -----//
    //-------------------------------------------------//

    //Loops through all streets and stores the ids of its intersections
    for (unsigned i = 0; i < num_of_streets; ++i) {
        std::vector<unsigned> segment_id = street_id_to_street_segments_map[i];
        unsigned num_of_segment_id = segment_id.size();

        for (unsigned j = 0; j < num_of_segment_id; ++j) {
            street_ids_to_intersection_ids[i].push_back(getStreetSegmentInfo(segment_id[j]).from);
            street_ids_to_intersection_ids[i].push_back(getStreetSegmentInfo(segment_id[j]).to);
        }

        std::sort(street_ids_to_intersection_ids[i].begin(), street_ids_to_intersection_ids[i].end());
        street_ids_to_intersection_ids[i].erase(std::unique(street_ids_to_intersection_ids[i].begin(),
                street_ids_to_intersection_ids[i].end()),
                street_ids_to_intersection_ids[i].end());
    }

    //---------------------------------------------------//
    //--SETUP: street_segment_id_to_curve_point_lengths--//
    //---------------------------------------------------//

    //Loops through all street segments and stores the lengths of all its curve points
    for (unsigned i = 0; i < num_of_street_segments; ++i) {
        StreetSegmentInfo segment_info = getStreetSegmentInfo(i);

        unsigned num_of_curve_points = segment_info.curvePointCount;

        //vector that stores curve point lengths for each street segment
        street_segment_id_to_curve_point_lengths[i] = std::vector<double>();

        //vector that stores LatLon points for each curve point
        std::vector<LatLon> curve_points = street_segment_id_to_curve_points[i];

        //If there are no curve points
        if (num_of_curve_points == 0) {
            street_segment_id_to_curve_point_lengths[i].push_back(find_distance_between_two_points(getIntersectionPosition(segment_info.from),
                    getIntersectionPosition(segment_info.to)));
        } else {
            //Loops through all curve points
            for (unsigned j = 0; j < num_of_curve_points; ++j) {
                if (j == 0) {
                    street_segment_id_to_curve_point_lengths[i].push_back(find_distance_between_two_points(getIntersectionPosition(segment_info.from), curve_points[j]));
                } else
                    street_segment_id_to_curve_point_lengths[i].push_back(find_distance_between_two_points(curve_points[j], curve_points[j - 1]));
            }
            street_segment_id_to_curve_point_lengths[i].push_back(find_distance_between_two_points(curve_points[num_of_curve_points - 1], getIntersectionPosition(segment_info.to)));
        }
    }

    //-------------------------------------------------//
    //---- SETUP: street_segment_ids_to_street_name ---//
    //-------------------------------------------------//

    //Loops through street segments and stores name of corresponding street
    for (unsigned i = 0; i < num_of_street_segments; ++i) {
        StreetSegmentInfo segment_info = getStreetSegmentInfo(i);

        unsigned streetID = segment_info.streetID;
        std::string street_name = getStreetName(streetID);
        street_segment_id_to_street_name.push_back(street_name);
    }

    //--------------------------------------------------//
    //----- SETUP: street_segment_ids_to_street_id -----//
    //--------------------------------------------------//
    for (unsigned i = 0; i < num_of_street_segments; ++i){
        StreetSegmentInfo segment_info = getStreetSegmentInfo(i);
        unsigned street_ID = segment_info.streetID;
        street_segment_id_to_street_id.push_back(street_ID);   
    }
    
    
    
    
    //-----------------------------------------------------//
    //----- SETUP: intersection_id_to_cartesian_points-----//
    //-----------------------------------------------------//

    //Loops through all intersections and stores its Cartesian coords
    //for (unsigned i = 0; i < num_of_intersections; ++i)
     //   intersection_id_to_cartesian_point.push_back(to_cartesian(getIntersectionPosition(i)));
    
    //----------------------------------------//
    //----- SETUP: POI_name_to_POI_indexes-----//
    //----------------------------------------//
    
    for(unsigned i = 0; i < getNumberOfPointsOfInterest(); ++i)
        POI_name_to_POI_indexes[getPointOfInterestName(i)].push_back(i);          
    
    //----------------------------------------------------------//
    //--------------------SETUP: travel_time--------------------//
    //----------------------------------------------------------//

    //Loops through all street segments and sums travel times
    //unsigned num_of_street_segments = getNumberOfStreetSegments();


    for (unsigned i = 0; i < num_of_street_segments; ++i) {
        double distance = find_street_segment_length(i);
        double time = distance / (getStreetSegmentInfo(i).speedLimit / 3.6); // (m/s)
        travel_time.push_back(time);
    }
    
    //-----------------------------------------------------//
    //------------------- SETUP: rtrees -------------------//
    //-----------------------------------------------------//

    unsigned number_of_POI = getNumberOfPointsOfInterest();

    for (unsigned i = 0; i < number_of_POI; ++i) {
        std::vector<double> points_xyz = to_cartesian_3D(getPointOfInterestPosition(i));
        point p(points_xyz[0], points_xyz[1], points_xyz[2]);

        PositionTree.insert(std::make_pair(p, i));
    }

    for (unsigned i = 0; i < num_of_intersections; ++i) {
        std::vector<double> points_xyz = to_cartesian_3D(getIntersectionPosition(i));
        point p(points_xyz[0], points_xyz[1], points_xyz[2]);

        IntersectionTree.insert(std::make_pair(p, i));
    }   
    
    //------------------------------------------------------------//
    //------------------- SETUP: adjacencyList -------------------//
    //------------------------------------------------------------//
        
    for(unsigned i = 0; i < getNumberOfIntersections(); ++i)
    {
        // all legal adjacent intersections
        std::vector<unsigned> neighbor_nodes = find_adjacent_intersections(i);                     
        std::vector<IntersectionNode> tempVector;
             
        for(unsigned j = 0; j < neighbor_nodes.size(); ++j)
        {
            IntersectionNode node;
            node.nodeID = neighbor_nodes[j];
            
            std::vector<unsigned> street_seg = intersection_id_to_street_segments[node.nodeID];

            for (unsigned k = 0; k < street_seg.size(); ++k)
            {
                if (getStreetSegmentInfo(street_seg[k]).oneWay && (getStreetSegmentInfo(street_seg[k]).to == node.nodeID) && (getStreetSegmentInfo(street_seg[k]).from == i)) 
                {
                    node.streetSegments.push_back(street_seg[k]); // push back the street segment id
                    node.travelTimes.push_back(travel_time[street_seg[k]]);
                } 
                 
                if (!getStreetSegmentInfo(street_seg[k]).oneWay &&
                   ((getStreetSegmentInfo(street_seg[k]).from == node.nodeID) && (getStreetSegmentInfo(street_seg[k]).to == i)) ||
                   ((getStreetSegmentInfo(street_seg[k]).from == i) && (getStreetSegmentInfo(street_seg[k]).to == node.nodeID))) 
                {
                    node.streetSegments.push_back(street_seg[k]);
                    node.travelTimes.push_back(travel_time[street_seg[k]]);
                }                      
            }
            tempVector.push_back(node);
        } 
        adjacencyList.push_back(tempVector);
    }
}