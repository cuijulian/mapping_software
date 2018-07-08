#pragma once

/*************************/
/***** INCLUDE FILES *****/
/*************************/
#include <vector>
#include <unordered_map>

#include "StreetsDatabaseAPI.h"
#include "IntersectionNode.h"

#include <boost/geometry.hpp>              // RTree Library
#include <boost/geometry/index/rtree.hpp>  // RTree Library

/***********************************/
/***** DEFINE RTREE ATTRIBUTES *****/
/***********************************/
namespace bg  = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<double, 3, bg::cs::cartesian> point; // define a point for rtree use
typedef bg::model::box<point> box;
typedef std::pair<point, unsigned> value;                     // couple point and unsigned ID together
typedef bgi::rtree< value, bgi::rstar<8> > rtree;

/*****************************************************/
/***** DEFINE GLOBAL DATA STRUCTURES + VARIABLES *****/
/*****************************************************/
extern std::unordered_map<std::string, std::vector<unsigned>> street_name_to_id_map;               // streetName      -> streetIDs
extern std::unordered_map<unsigned, std::vector<unsigned>> street_id_to_street_segments_map;       // streetID        -> <streetSegments>
extern std::unordered_map<unsigned, std::vector<unsigned>> intersection_id_to_street_segments;     // intersectionID  -> <streetSegments>
extern std::unordered_map<unsigned, std::vector<LatLon>> street_segment_id_to_curve_points;        // streetSegmentID -> <LatLon points>
extern std::vector <LatLon> intersection_id_to_position;   
extern std::unordered_map<unsigned, std::vector<unsigned>> street_ids_to_intersection_ids;         // streetID        -> <intersectionIDs>
extern std::vector <unsigned>  street_segment_id_to_street_id;                                // streetSegmentID -> streetID
extern std::unordered_map<unsigned, std::vector<double>> street_segment_id_to_curve_point_lengths; // streetSegmentID -> <distances>
extern std::vector <std::string> street_segment_id_to_street_name;                                 // streetSegmentID -> streetName
extern std::vector <double> travel_time;                                                           // streetSegmentID -> travel time
extern std::unordered_map <std::string, std::vector<unsigned>> POI_name_to_POI_indexes;            // POI_name        -> POI_index(s)
extern rtree IntersectionTree;                                                                     // rtree for intersections
extern rtree PositionTree;                                                                         // rtree for POI
extern std::vector<std::vector<IntersectionNode>> adjacencyList;

extern double average_lat;

/***********************************/
/***** DEFINE USEFUL FUNCTIONS *****/
/***********************************/
//Converts a LatLon point into Cartesian system.
//Returns a vector of x, y, and z coords.
std::vector<double> to_cartesian_3D(LatLon point_);
std::vector<unsigned> street_segments_in_common (const unsigned intersect_id_1, const unsigned intersect_id_2);
std::vector<double> find_time_between_intersections(const unsigned intersect_id_start, const unsigned intersect_id_end);

/*************************************************/
/***** FUNCTION FILLS GLOBAL DATA STRUCTURES *****/
/*************************************************/
void initialize_global_database();