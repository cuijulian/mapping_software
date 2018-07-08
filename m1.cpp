/* 
 * Copyright 2018 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated 
 * documentation files (the "Software") in course work at the University 
 * of Toronto, or for personal use. Other uses are prohibited, in 
 * particular the distribution of the Software either publicly or to third 
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
//#include "StreetsDatabaseAPI.h"
#include "m1.h"
#include "SearchBar.cpp"

#include "Global.h"

#include <unordered_map>
#include <math.h>
#include <algorithm>    // std::min_element
#include <iterator>     // std::begin, std::end
#include <climits>

// rtree libs + initialization
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/foreach.hpp>

//#include <boost/geometry.hpp>              // RTree Library
//#include <boost/geometry/index/rtree.hpp>  // RTree Library

SearchBar *search_bar;


//------------------------------------------------------//
//----------GLOBAL DATA STRUCTURES + VARIABLES----------//
//------------------------------------------------------//
//rtree *PositionTree;         // rtree for POI
//rtree *IntersectionTree;     // rtree for intersections

//bool loaded_once = false;

//------------------------------------------------------//
//---------------FUNCTION IMPLEMENTATIONS---------------//
//------------------------------------------------------//

//Loads a map streets.bin file. Returns true if successful, false if some error
//occurs and the map can't be loaded.
bool
load_map(std::string map_path) 
{        
    bool loaded_successful = false;

    if (loadStreetsDatabaseBIN(map_path)) loaded_successful = true;

    // successfully loaded the map
    if (/*!loaded_once &&*/ loaded_successful) {
            
        initialize_global_database();
        
        //---------------------------------------------------------//
        //------------------- SETUP: Search Bar -------------------//
        //---------------------------------------------------------//
        
        double x1 = 0;
        double y2 = 50;
        double y1 = 0;
        double x2 = 200;

        search_bar = new SearchBar(x1, y1, x2, y2);
        
                //loaded_once = true;
        return true;   
    }
    
    if(loaded_successful) return true;
    
    return false;
}

//Frees memory and closes the map.
void
close_map() 
{
    delete search_bar;
    //delete data;
    //delete PositionTree;
    //delete IntersectionTree;
    closeStreetDatabase();
}


//Returns street id(s) for the given street name
//If no street with this name exists, returns a 0-length vector.
std::vector<unsigned>
find_street_ids_from_name(std::string street_name) // streetID -> streetName
{
    return street_name_to_id_map[street_name];
}


//Returns the street segments for the given intersection 
std::vector<unsigned>
find_intersection_street_segments(unsigned intersection_id) // intersectionID -> <streetSegments>
{
    return intersection_id_to_street_segments[intersection_id];
}


//Returns the street names at the given intersection (includes duplicate street 
//names in returned vector)
std::vector<std::string>
find_intersection_street_names(unsigned intersection_id) // intersectionID -> <streetNames>
{
    // get all street segments connected to an intersection
    std::vector<unsigned>    street_segments_at_intersection = intersection_id_to_street_segments[intersection_id];
    std::vector<std::string> street_names;
    std::vector<unsigned>    street_ids;
    
    // loop through each street_segment_at_intersection and push_back
    for(unsigned i = 0; i < street_segments_at_intersection.size(); ++i) 
        street_ids.push_back(getStreetSegmentInfo(street_segments_at_intersection[i]).streetID);
    
    for(unsigned i = 0; i < street_ids.size(); ++i) 
        street_names.push_back(getStreetName(street_ids[i]));
    
    return street_names;
}


//Returns true if you can get from intersection1 to intersection2 using a single 
//street segment (hint: check for 1-way streets too)
//corner case: an intersection is considered to be connected to itself
bool
are_directly_connected(unsigned intersection_id1, unsigned intersection_id2) // use the map of intersections to segments
{
    if (intersection_id1 == intersection_id2) return true;
   
    std::vector<unsigned>::iterator it;    
    std::vector<unsigned> street_segment_ids1 = find_intersection_street_segments(intersection_id1);
    
    for (it=street_segment_ids1.begin() ; it!=street_segment_ids1.end(); ++it) 
    {
        if (getStreetSegmentInfo(*it).oneWay) 
        {
            if ((getStreetSegmentInfo(*it).from == intersection_id1) && (getStreetSegmentInfo(*it).to == intersection_id2))
                return true;
        } 
        else 
        {
            if (((getStreetSegmentInfo(*it).from == intersection_id1) && (getStreetSegmentInfo(*it).to   == intersection_id2)) ||
                ((getStreetSegmentInfo(*it).to   == intersection_id1) && (getStreetSegmentInfo(*it).from == intersection_id2)))
                return true;
        }
    }
    
    return false;
}


//Returns all intersections reachable by traveling down one street segment 
//from given intersection (hint: you can't travel the wrong way on a 1-way street)
//the returned vector should NOT contain duplicate intersections
std::vector<unsigned>
find_adjacent_intersections(unsigned intersection_id) 
{
    std::vector<unsigned> adjacent_intersections;
    std::vector<unsigned> street_seg = intersection_id_to_street_segments[intersection_id];
    
    unsigned seg_count = street_seg.size();

    for (unsigned i = 0; i < seg_count; i++)
    {
        if (getStreetSegmentInfo(street_seg[i]).from == intersection_id) 
        {
            adjacent_intersections.push_back(getStreetSegmentInfo(street_seg[i]).to); // push back the intersection id
        } 
        else 
        {
            if (!getStreetSegmentInfo(street_seg[i]).oneWay)
                adjacent_intersections.push_back(getStreetSegmentInfo(street_seg[i]).from);
        }
    }
    
    std::sort(adjacent_intersections.begin(), adjacent_intersections.end());     
    adjacent_intersections.erase(std::unique(adjacent_intersections.begin(), adjacent_intersections.end()), adjacent_intersections.end());

    return adjacent_intersections;
}


//Returns all street segments for the given street
std::vector<unsigned>
find_street_street_segments(unsigned street_id) 
{
    return street_id_to_street_segments_map[street_id];
}


//Returns all intersections along the a given street
std::vector<unsigned> 
find_all_street_intersections(unsigned street_id) 
{
    return street_ids_to_intersection_ids[street_id];
}


//Return all intersection ids for two intersecting streets
//This function will typically return one intersection id.
//However street names are not guarenteed to be unique, so more than 1 intersection id
//may exist
std::vector<unsigned>
find_intersection_ids_from_street_names(std::string street_name1, std::string street_name2) 
{ 
    std::vector<unsigned> intersection_ids;
    std::vector<unsigned> intersections1, intersections2;
    std::vector<unsigned> street_id1 = street_name_to_id_map[street_name1];
    std::vector<unsigned> street_id2 = street_name_to_id_map[street_name2];
    
    unsigned street_id1_size=street_id1.size();
    unsigned street_id2_size=street_id2.size();
    
    for(unsigned i=0; i<street_id1_size; ++i)
    {
        intersections1 = find_all_street_intersections(street_id1[i]);
        for(unsigned j=0; j<street_id2_size; ++j)
        {
            intersections2=find_all_street_intersections(street_id2[j]);
            std::set_intersection(intersections1.begin(),intersections1.end(),intersections2.begin(),intersections2.end(),std::back_inserter(intersection_ids));
        }
    }
    
    return intersection_ids;
}


//Returns the distance between two coordinates in meters
double
find_distance_between_two_points(LatLon point1, LatLon point2) 
{
    double point1_lat_rad = point1.lat() * DEG_TO_RAD;
    double point1_lon_rad = point1.lon() * DEG_TO_RAD;
    double point2_lat_rad = point2.lat() * DEG_TO_RAD;
    double point2_lon_rad = point2.lon() * DEG_TO_RAD;
    
    double latitudeAverage = (point1_lat_rad + point2_lat_rad)/2.0;
    
    double x1 = point1_lon_rad*cos(latitudeAverage);
    double x2 = point2_lon_rad*cos(latitudeAverage);
    
    double y1 = point1_lat_rad;
    double y2 = point2_lat_rad;

    return EARTH_RADIUS_IN_METERS * sqrt(pow((y2 - y1), 2) + pow((x2 - x1), 2));
}


//Returns the length of the given street segment in meters
double
find_street_segment_length(unsigned street_segment_id) 
{
    double distance = 0;
    std::vector<double> curve_points = street_segment_id_to_curve_point_lengths[street_segment_id];
    unsigned num_of_elements = curve_points.size();
    
    for (unsigned i = 0; i < num_of_elements; ++i)
        distance += curve_points[i];
    
    return distance;
}


//Returns the length of the specified street in meters
double
find_street_length(unsigned street_id) 
{
    std::vector<unsigned> street_segment_ids = street_id_to_street_segments_map[street_id];
    double total_distance = 0;
    unsigned street_segments_ids_size = street_segment_ids.size();
    
    for (unsigned i = 0; i < street_segments_ids_size; ++i)
        total_distance += find_street_segment_length(street_segment_ids[i]);

    return total_distance;
}


//Returns the travel time to drive a street segment in seconds 
//(time = distance/speed_limit)
double
find_street_segment_travel_time(unsigned street_segment_id) 
{
    return travel_time[street_segment_id];
}


//Returns the nearest point of interest to the given position
unsigned
find_closest_point_of_interest(LatLon my_position) 
{
    std::vector<value>  result;
    std::vector<double> points_xyz = to_cartesian_3D(my_position);
    
    point my_position_point (points_xyz[0], points_xyz[1], points_xyz[2]);
    //query the PositionTree to find the nearest point to my_position
    PositionTree.query(bgi::nearest(my_position_point, 1), std::back_inserter(result));
        
    return result[0].second;
}


//Returns the the nearest intersection to the given position
unsigned
find_closest_intersection(LatLon my_position) 
{
    std::vector<value>  result;
    std::vector<double> points_xyz = to_cartesian_3D(my_position);
    
    point my_position_point (points_xyz[0], points_xyz[1], points_xyz[2]);
    // query the IntersectionTree to find the nearest point to my_position
    IntersectionTree.query(bgi::nearest(my_position_point, 1), std::back_inserter(result));
        
    return result[0].second;
}
