#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "graphics.h"
#include "SearchBar.cpp"
#include <string>
#include <sstream> 
#include "Global.h"

#include "graphics_types.h"
#include <X11/keysym.h>

#include <unordered_map>
#include <vector>
#include <iostream>
#include <math.h>
#include <algorithm>    
#include <iterator>     
#include <climits>

#include "OSMEntity.h"
#include "OSMNode.h"
#include "OSMWay.h"
#include "OSMRelation.h"
#include "OSMDatabaseAPI.h"

// rtree libs + initialization
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/foreach.hpp>

// namespaces for rtree's
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// rtree setup for Intersection & POI 
typedef bg::model::point <double, 2, bg::cs::cartesian> point2;
typedef bg::model::box<point2> box2;
typedef std::pair <point2, unsigned> value2;
typedef bgi::rtree <value2, bgi::rstar < 8 >> rtree2;
rtree2 *g_IntersectionTree;
rtree2 *g_POITree;

// rtree setup for Features
typedef bg::model::polygon<point2, false, false> polygon;
typedef std::pair <box2, unsigned> value_polygon;
typedef bgi::rtree <value_polygon, bgi::rstar < 16, 4 >> polygonTree;
std::vector<polygon> *polygons;
polygonTree *g_PolygonTree;

// rtree setup for Street Segments
typedef std::pair <point2, unsigned> segmentValue;
typedef bgi::rtree <segmentValue, bgi::rstar < 8 >> streetSegmentTree;
streetSegmentTree *g_StreetSegmentTree;

//M1_Data *data2;

std::vector <std::vector<double>> intersection_id_to_cartesian_point; // intersectionID  -> cartesian point


//double average_lat = 0;
double original_world_area = 0;
bool g_find_intersection = false;
std::vector<unsigned> intersection_id_found;

t_bound_box world_box = get_visible_world();

bool click_intersection = false;
bool click_intersection_right = false;
unsigned click_intersection_id = 0;
unsigned click_intersection_id_right = 0;
bool click_POI = false;
unsigned click_POI_id = 0;


std::vector<double> *feature_areas;
std::vector<double> *feature_lengths;

double feature_areas_median = 0; // median feature area
double feature_lengths_median = 0; // median feature length
double feature_areas_mean = 0;
double feature_lengths_mean = 0;

Surface map_marker = load_png_from_file("/nfs/ug/homes-4/k/kornyeye/ece297/work/mapper/libstreetmap/resources/Marker20x20.png");
Surface one_way_marker = load_png_from_file("/nfs/ug/homes-4/k/kornyeye/ece297/work/mapper/libstreetmap/resources/Marker20x20.png");
Surface start_marker = load_png_from_file("/nfs/ug/homes-4/k/kornyeye/ece297/work/mapper/libstreetmap/resources/end.png");
Surface end_marker = load_png_from_file("/nfs/ug/homes-4/k/kornyeye/ece297/work/mapper/libstreetmap/resources/start.png");
double speed_limit_mean = 0;


// all street segment indexes for each street type
std::unordered_map<unsigned, std::string> *segment_id_to_street_type;
std::unordered_map<OSMID, unsigned> *OSMID_way_map;


bool first_time = true;
bool is_help = false;

//-------------------------------------------------//
//---------------FUNCTION PROTOTYPES---------------//
//-------------------------------------------------//

// Callback funcs
void drawscreen(void);
void act_on_mouse_button(float, float, t_event_buttonPressed);
void act_on_keypress(char c, int keysym);

// Drawing streets/segments
void draw_curve(unsigned);
void draw_street_segment(unsigned street_segment_id);
void draw_all_streets_in_box(t_bound_box t_box);

//Drawing street names
void draw_all_street_names(double zoom_level);
void draw_street_name(unsigned street_segment_id, double zoom_level,
        unsigned speed_limit, t_bound_box segment_box);

// Drawing intersections 
void highlight_intersection(unsigned);
void draw_intersection(unsigned);
void draw_all_intersections();
void draw_all_intersections_in_box(t_bound_box t_box);

// Drawing features
void draw_feature(unsigned);
void draw_all_features_in_box(t_bound_box t_box);
void draw_all_features();

// Drawing POI
void draw_POI(unsigned);
void draw_all_POI();
void draw_all_POI_in_box(t_bound_box t_box);
void draw_all_POI_names(double zoom_level);

// Helper func(s)
//Converts LatLon point to Cartesian coords
//Returns vector of x, y coords
std::vector<double>to_cartesian(LatLon point_);
std::vector<double> get_polygon_area_or_length(unsigned feature_id);
LatLon to_LatLon(float x, float y);
void find_func(void (*drawscreen) (void));
void act_on_find_button_func(void (*drawscreen_ptr) (void));
void act_on_switch_button_func(void (*drawscreen_ptr) (void));
void act_on_find_path_button_func(void (*drawscreen_ptr) (void));
void act_on_help_button_func(void (*drawscreen_ptr) (void));

void fill_OSMID_way_map();
std::string type_of_highway_street(unsigned street_segment_id);
void insert_streets_into_vectors();
void draw_intersection_info(unsigned intersection_id);
void draw_POI_info(unsigned click_POI_id);
void highlight_street(std::vector<unsigned> path);
std::vector<unsigned> shortest_path;
bool path_find = false;
bool searchbar = true;
bool first_string = true;
bool second_string = false;
bool ready_to_search = false;
std::string first_name;
std::string second_name;
unsigned enter_times = 0;
unsigned first_inter_id = 0;
unsigned second_inter_id = 0;
void draw_search_bar();
void show_path();
void print_directions();

std::vector< char > search_string_first;
std::vector< char > search_string_second;
void draw_search_bar_text();
t_bound_box search_box_blank;
bool poi_path = false;

std::vector<std::string> map_paths = {
    "/cad2/ece297s/public/maps/beijing_china.streets.bin",
    "/cad2/ece297s/public/maps/cape-town_south-africa.streets.bin",
    "/cad2/ece297s/public/maps/hamilton_canada.streets.bin",
    "/cad2/ece297s/public/maps/tokyo_japan.streets.bin",
    "/cad2/ece297s/public/maps/hong-kong_china.streets.bin",
    "/cad2/ece297s/public/maps/interlaken_switzerland.streets.bin",
    "/cad2/ece297s/public/maps/london_england.streets.bin",
    "/cad2/ece297s/public/maps/moscow_russia.streets.bin",
    "/cad2/ece297s/public/maps/new-dehli_india.streets.bin",
    "/cad2/ece297s/public/maps/new-york_usa.streets.bin",
    "/cad2/ece297s/public/maps/rio-de-janeiro_brazil.streets.bin",
    "/cad2/ece297s/public/maps/singapore.streets.bin",
    "/cad2/ece297s/public/maps/sydney_australia.streets.bin",
    "/cad2/ece297s/public/maps/tehran_iran.streets.bin",
    "/cad2/ece297s/public/maps/cairo_egypt.streets.bin",
    "/cad2/ece297s/public/maps/golden-horseshoe_canada.streets.bin",
    "/cad2/ece297s/public/maps/iceland.streets.bin",
    "/cad2/ece297s/public/maps/saint-helena.streets.bin",
    "/cad2/ece297s/public/maps/toronto_canada.streets.bin"
};
int current_map_index = map_paths.size() - 1;


//----------------------------------------------------------------//
//--------------------FUNCTION IMPLEMENTATIONS--------------------//
//----------------------------------------------------------------//

/* respond to key_pressed (could be arrow keys) */

//Loads all map information data structures
//Initializes graphics
//Closes map and OSM database when finished

void
draw_map() {
    /**
    @param:  
    @func: (1) set up OSM data for given map
           (2) intialize required data structures (e.g. all rtree's)
           (3) initilize many of the global vars
           (4) set up graphics window
           (5) clear/delete many data structs utilized
     */
    std::string osm_bin_path = map_paths[current_map_index].erase(map_paths[current_map_index].size() - 11, 11);
    osm_bin_path.append("osm.bin");
    std::cout << osm_bin_path << '\n';
    loadOSMDatabaseBIN(osm_bin_path);

    //----------------------------------------------------//
    //----- SETUP: average_lat & original world view -----//
    //----------------------------------------------------//

    double max_lat = getIntersectionPosition(0).lat(); // lat - y
    double min_lat = max_lat;

    double max_lon = getIntersectionPosition(0).lon(); // lon - x
    double min_lon = max_lon;

    for (unsigned i = 0; i < getNumberOfIntersections(); ++i) {
        max_lat = std::max(max_lat, getIntersectionPosition(i).lat()); // lat
        min_lat = std::min(min_lat, getIntersectionPosition(i).lat());

        max_lon = std::max(max_lon, getIntersectionPosition(i).lon()); // lon
        min_lon = std::min(min_lon, getIntersectionPosition(i).lon());
    }

    average_lat = 0.5 * (max_lat * DEG_TO_RAD + min_lat * DEG_TO_RAD);


    unsigned num_of_intersections = getNumberOfIntersections();
    for (unsigned i = 0; i < num_of_intersections; ++i)
        intersection_id_to_cartesian_point.push_back(to_cartesian(getIntersectionPosition(i)));

    //---------------------------------//
    //----- SETUP: M1_Data Object -----//
    //---------------------------------//

    unsigned num_of_streets = getNumberOfStreets();
    unsigned num_of_street_segments = getNumberOfStreetSegments();

    //data2 = new M1_Data; // must be initialized here since can't initialize it before the map data has been loaded

    g_IntersectionTree = new rtree2;
    g_POITree = new rtree2;
    g_PolygonTree = new polygonTree;
    g_StreetSegmentTree = new streetSegmentTree;
    segment_id_to_street_type = new std::unordered_map<unsigned, std::string>;
    OSMID_way_map = new std::unordered_map<OSMID, unsigned>;
    feature_areas = new std::vector<double>;
    feature_lengths = new std::vector<double>;
    polygons = new std::vector<polygon>;

    //-----------------------------------------------------//
    //--------------------SETUP: rtrees--------------------//
    //-----------------------------------------------------//

    //Inserts all intersection positions and IDs into rtree
    for (unsigned i = 0; i < num_of_intersections; ++i) {
        std::vector<double> points_xy = to_cartesian(getIntersectionPosition(i));

        point2 p(points_xy[0], points_xy[1]);

        g_IntersectionTree->insert(std::make_pair(p, i));
    }

    //Inserts all POI positions and IDs into rtree
    unsigned num_of_POI = getNumberOfPointsOfInterest();

    for (unsigned i = 0; i < num_of_POI; ++i) {
        std::vector<double> points_xy = to_cartesian(getPointOfInterestPosition(i));

        point2 p(points_xy[0], points_xy[1]);

        g_POITree->insert(std::make_pair(p, i));
    }

    //Inserts all street segments position and IDs into rtree
    for (unsigned i = 0; i < num_of_street_segments; ++i) {

        //Gets info of the street segment
        StreetSegmentInfo segment_info = getStreetSegmentInfo(i);
        unsigned intersection_from = segment_info.from;

        std::vector<double> from_xy = to_cartesian(getIntersectionPosition(intersection_from));

        point2 p(from_xy[0], from_xy[1]);

        g_StreetSegmentTree->insert(std::make_pair(p, i));
    }

    //-----------------------------------------------------------------//
    //--------------------SETUP: rtree for polygons--------------------//
    //-----------------------------------------------------------------//

    unsigned num_of_features = getNumberOfFeatures();

    feature_areas->resize(num_of_features);
    feature_lengths->resize(num_of_features);

    std::fill(feature_areas->begin(), feature_areas->end(), 0);
    std::fill(feature_lengths->begin(), feature_lengths->end(), 0);


    std::vector<double> temp_areas;
    std::vector<double> temp_lengths;

    //Inserts vector of points of feature locations and IDs into rtree
    for (unsigned i = 0; i < num_of_features; ++i) {
        unsigned num_of_points = getFeaturePointCount(i);
        std::vector<LatLon> coordinates;
        polygon p;

        for (unsigned j = 0; j < num_of_points; ++j)
            coordinates.push_back(getFeaturePoint(i, j));

        for (unsigned j = 0; j < num_of_points; ++j)
            p.outer().push_back(point2(to_cartesian(coordinates[j])[0],
                to_cartesian(coordinates[j])[1]));

        polygons->push_back(p);

        //---------------//

        std::vector<double> area_length = get_polygon_area_or_length(i);

        // fill feature_areas & feature_lengths vectors
        // feature_area/feature_lengths vector index corresponds to feature area/length
        if (area_length[0] != 0) {
            (*feature_areas)[i] = area_length[0];
            temp_areas.push_back(area_length[0]);
        } else {
            (*feature_lengths)[i] = area_length[1];
            temp_lengths.push_back(area_length[1]);
        }
    }

    for (unsigned i = 0; i < polygons->size(); ++i) // i corresponds to featureID
    {
        box2 b = bg::return_envelope<box2>((*polygons)[i]);
        g_PolygonTree->insert(std::make_pair(b, i));
    }

    //
    //
    //

    unsigned temp_areas_size = temp_areas.size();

    for (unsigned i = 0; i < temp_areas_size; ++i)
        feature_areas_mean += temp_areas[i];

    feature_areas_mean /= temp_areas_size;

    unsigned temp_lengths_size = temp_lengths.size();

    for (unsigned i = 0; i < temp_lengths_size; ++i)
        feature_lengths_mean += temp_lengths[i];

    feature_lengths_mean /= temp_lengths_size;

    //Average speed limit of street segments
    for (unsigned i = 0; i < num_of_street_segments; ++i) {
        StreetSegmentInfo segment_info = getStreetSegmentInfo(i);
        speed_limit_mean += segment_info.speedLimit;
    }

    speed_limit_mean = speed_limit_mean / num_of_street_segments;


    //------------------------------------------------------------------------//


    //-----------------------------------------------------//    
    //---------------INITIALIZE THE GRAPHICS---------------//
    //-----------------------------------------------------//    

    // initialize graphics window with title "Campus Map" and background

    if (first_time) {
        t_color background_colour(220, 220, 220, 255);
        init_graphics("Campus Maps", background_colour);
        update_message("Interactive graphics");
        // create "Find" button & "Switch Map" button
        create_button("Window", "Find", act_on_find_button_func);
        create_button("Find", "Switch", act_on_switch_button_func);
        create_button("Find", "POI Path", act_on_find_path_button_func);
        create_button("Find", "Help", act_on_help_button_func);
        first_time = false;
    }

    //Finds bounds of visible map
    double max_y = max_lat * DEG_TO_RAD;
    double max_x = max_lon * cos(average_lat) * DEG_TO_RAD;
    double min_y = min_lat * DEG_TO_RAD;
    double min_x = min_lon * cos(average_lat) * DEG_TO_RAD;

    // setup original world view
    set_visible_world(min_x, min_y, max_x, max_y);
    original_world_area = get_visible_world().area();

    //
    //
    //
    fill_OSMID_way_map();
    insert_streets_into_vectors();
    //
    //
    //

    set_keypress_input(searchbar);

    if (!first_time) drawscreen();
    // calls <= 4 functions repeatedly in a loop (returns when user exits)
    event_loop(act_on_mouse_button, NULL, act_on_keypress, drawscreen);

    destroy_button("Switch");
    destroy_button("Find");
    destroy_button("POI Path");
    destroy_button("Help");

    //delete data2;
    delete g_IntersectionTree;
    delete g_POITree;
    delete g_PolygonTree;
    delete g_StreetSegmentTree;
    delete segment_id_to_street_type;
    delete OSMID_way_map;
    delete feature_areas;
    delete feature_lengths;
    delete polygons;

    closeOSMDatabase();

    // close graphics window
    close_graphics();
}





//----------------------------//
//-----CALLBACK FUNCTIONS-----//
//----------------------------//

void act_on_keypress(char c, int keysym) {
    // function to handle keyboard press event, the ASCII character is returned
    // along with an extended code (keysym) on X11 to represent non-ASCII
    // characters like the arrow keys.



    //#ifdef X11 // Extended keyboard codes only supported for X11 for now
    switch (keysym) {
        case XK_Left:
            std::cout << "Left Arrow" << std::endl;
            break;
        case XK_Right:
            std::cout << "Right Arrow" << std::endl;
            break;
        case XK_Up:
            std::cout << "Up Arrow" << std::endl;
            break;
        case XK_Down:
            std::cout << "Down Arrow" << std::endl;
            break;

        case XK_Return:
            std::cout << "Enter button" << std::endl;
            enter_times = enter_times + 1;
            if (enter_times % 2 == 1) {
                first_string = false;
                second_string = true;
                ready_to_search = false;
                std::string search_text_temp(search_string_first.begin(), search_string_first.end());
                first_name = search_text_temp;
                search_string_first.clear();
                
            } else {
                first_string = true;
                second_string = false;
                ready_to_search = true;
                std::string search_text_temp(search_string_second.begin(), search_string_second.end());
                second_name = search_text_temp;
                search_string_second.clear();
            }
            std::cout << enter_times << std::endl;
            drawscreen();
            //   std::string first_temp(search_string.begin(), search_string.end());
            //   first_name=first_temp;
            break;

        case XK_Caps_Lock:
            break;
        case XK_Shift_L:
            break;
        case XK_Shift_R:
            break;
            /* case XK_KP_Space:
                 first_string=true;
                 second_string=false;
                 ready_to_search=true;
                 drawscreen();
                 break;*/
        case XK_BackSpace:
            if (first_string) {
                search_string_first.erase(search_string_first.begin() + search_string_first.size() - 1);
            }
            if (second_string) {
                search_string_second.erase(search_string_second.begin() + search_string_second.size() - 1);
            }
            drawscreen();
            break;
        default:
            ready_to_search = false;
            if (first_string) {
                search_string_first.push_back(c);
                for (unsigned i = 0; i < search_string_first.size(); ++i) {
                    std::cout << search_string_first[i];
                }
            }
            if (second_string) {
                search_string_second.push_back(c);
                for (unsigned j = 0; j < search_string_second.size(); ++j) {
                    std::cout << search_string_second[j];
                }
            }
            drawscreen();
            std::cout << "Key press: char is " << c << std::endl;
            break;
    }
    //#endif
}






// highlight the path when the shortest path is given as street segments id

void draw_search_bar() {
    //set_coordinate_system(GL_WORLD);
    setcolor(WHITE);

    /*t_bound_box current(get_visible_world());
    float x1, y1, x2, y2;
    x1 = current.bottom_left().x;
    y1 = current.get_ycenter();
    x2 = current.get_xcenter();
    y2 = current.top_right().y;
    t_bound_box search_box1(x1, y1, x2, y2);
    // unsigned y3= search_box1.get_ycenter();
    t_bound_box search_box2(x1, search_box1.get_ycenter(), x2, y2);
    // unsigned y4= search_box2.get_ycenter();
    t_bound_box search_box3(x1, search_box2.get_ycenter(), x2, y2);
    // unsigned y5= search_box3.get_ycenter();  
    t_bound_box search_box4(x1, search_box3.get_ycenter(), x2, y2);

    fillrect(search_box4);
    search_box_blank = search_box4;*/

    t_point point1(0, 50); // bottom_left
    t_point xy1 = scrn_to_world(point1);

    t_point point2(450, 0); // top-right
    t_point xy2 = scrn_to_world(point2);
    t_bound_box search_bar_box(point1, point2);
    // setcolor(WHITE);
    fillrect(xy1.x, xy1.y, xy2.x, xy2.y);
    std::cout << "finish blank search bar\n";


    set_coordinate_system(GL_SCREEN);
    setcolor(BLACK);
    setfontsize(16);
    t_bound_box current(get_visible_world());
    if (first_string) {
        std::string search_text(search_string_first.begin(), search_string_first.end());
        drawtext(150, 30, search_text);
        std::cout << "first text\n";
    }// std:: cout<< search_text<< std:: endl;
    else if (second_string) {
        std::string search_text_second(search_string_second.begin(), search_string_second.end());
        drawtext(150, 30, search_text_second);
        std::cout << "second text\n";
    }
    set_coordinate_system(GL_WORLD);
}

void show_path() {


    if (!poi_path) {
        std::string street_name11;
        std::string temp;
        std::string street_name12;
        std::string street_name21;
        std::string street_name22;
        std::stringstream linestream1(first_name);
        std::stringstream linestream2(second_name);
        // linestream1 >> street_name11;
        street_name11.clear();
        std::cout << "in show path\n";
        bool first = true;
        linestream1 >> temp;
        // while not the last element 
        while (linestream1.tellg() != -1) {
            std::cout << " iN1\n";
            // linestream1 >> temp;
            if (temp != "&" && first) {
                if (street_name11 == "") {
                    street_name11 = street_name11 + temp;
                    linestream1 >> temp;
                } else {
                    street_name11 = street_name11 + " " + temp;
                    linestream1 >> temp;
                }
            } else if (temp != "&" && !first) {
                if (street_name12 == "") {
                    street_name12 = street_name12 + temp;
                    linestream1 >> temp;
                } else {
                    street_name12 = street_name12 + " " + temp;
                    linestream1 >> temp;
                }
            } else {
                first = false;
                linestream1>> temp;
            }
        }
        street_name12 = street_name12 + " " + temp;
        temp.clear();
        std::cout << " iN2\n";
        first = true;
        // linestream2 >> street_name21;
        linestream2 >> temp;
        while (linestream2.tellg() != -1) {
            // linestream2 >> temp;
            if (temp != "&" && first) {
                if (street_name21 == "") {
                    street_name21 = street_name21 + temp;
                    linestream2 >> temp;
                } else {
                    street_name21 = street_name21 + " " + temp;
                    linestream2 >> temp;
                }
            } else if (temp != "&" && !first) {
                if (street_name22 == "") {
                    street_name22 = street_name22 + temp;
                    linestream2 >> temp;
                } else {
                    street_name22 = street_name22 + " " + temp;
                    linestream2 >> temp;
                }
            }//when temp = &
            else {
                first = false;
                linestream2>> temp;
            }
        }
        street_name22 = street_name22 + " " + temp;
        temp.clear();

        std::cout << street_name11 << std::endl;
        std::cout << street_name12 << std::endl;
        std::cout << street_name21 << std::endl;
        std::cout << street_name22 << std::endl;
        // find the start intersection id and end intersection id
        std::vector<unsigned> start_intersection = find_intersection_ids_from_street_names(street_name11, street_name12);
        std::vector<unsigned> end_intersection = find_intersection_ids_from_street_names(street_name21, street_name22);
        //check if there is just one path
        std::cout << start_intersection.size() << std::endl;
        std::cout << end_intersection.size() << std::endl;


        std::vector<double> xy1 = intersection_id_to_cartesian_point[start_intersection[0]];
        double right, left, top, bottom;
        double x1 = xy1[0], y1 = xy1[1];
        double off = 0.0002;
        std::vector<double> xy2 = intersection_id_to_cartesian_point[end_intersection[0]];
        double x2 = xy2[0], y2 = xy2[1];
        if (x1 > x2) {
            right = x1;
            left = x2;
        } else {
            right = x2;
            left = x1;
        }

        if (y1 > y2) {
            bottom = y1;
            top = y2;
        } else {
            bottom = y2;
            top = y1;
        }

        set_visible_world(left - off, bottom - off, right + off, top + off);



        if (start_intersection.size() == 1 && end_intersection.size() == 1) {
            first_inter_id = start_intersection[0];
            second_inter_id = end_intersection[0];
            shortest_path = find_path_between_intersections(start_intersection[0], end_intersection[0], 0);
            
            std::cout << "11111111111111111111111 shortest path change" << std::endl;
            path_find = true;
            //highlight_street(shortest_path);
            drawscreen();
        } else if (start_intersection.size() == 0)
            std::cout << "Error: No start intersection found \n";
        else if (end_intersection.size() == 0)
            std::cout << "Error: No end intersection found \n";
        else
            std::cout << "Error: multiple path because of more than 2 intersections \n";
    }





    else {
        std::string street_name11;
        std::string temp;
        std::string street_name12;
        std::string poi_name;

        std::stringstream linestream1(first_name);
        // linestream1 >> street_name11;
        street_name11.clear();
        bool first = true;
        linestream1 >> temp;
        // while not the last element 
        while (linestream1.tellg() != -1) {
            // linestream1 >> temp;
            if (temp != "&" && first) {
                if (street_name11 == "") {
                    street_name11 = street_name11 + temp;
                    linestream1 >> temp;
                } else {
                    street_name11 = street_name11 + " " + temp;
                    linestream1 >> temp;
                }
            } else if (temp != "&" && !first) {
                if (street_name12 == "") {
                    street_name12 = street_name12 + temp;
                    linestream1 >> temp;
                } else {
                    street_name12 = street_name12 + " " + temp;
                    linestream1 >> temp;
                }
            } else {
                first = false;
                linestream1>> temp;
            }
        }
        street_name12 = street_name12 + " " + temp;
        temp.clear();
        poi_name = second_name;

        std::cout << street_name11 << std::endl;
        std::cout << street_name12 << std::endl;
        std::cout << poi_name << std::endl;
        // find the start intersection id and end intersection id
        std::vector<unsigned> start_intersection = find_intersection_ids_from_street_names(street_name11, street_name12);

        std::vector<double> xy1 = intersection_id_to_cartesian_point[start_intersection[0]];
        double x = xy1[0], y = xy1[1];
        double off = 0.0002;
        set_visible_world(x - off, y - off, x + off, y + off);


        //check if there is just one path
        if (start_intersection.size() == 0) {
            std::cout << "Error: No start intersection found \n";
        } else if (start_intersection.size() == 1) {
            shortest_path = find_path_to_point_of_interest(start_intersection[0], poi_name, 0);
            path_find = true;
            drawscreen();
        } else {
            std::cout << "ERROR" << std::endl;
        }

    }

}

void highlight_street(std::vector<unsigned> path) {
    t_color path_colour(0, 153, 204, 255);
    std::cout << "in highlightstreet \n";
    for (unsigned i = 0; i < path.size(); i++) {
        setcolor(path_colour);
        draw_street_segment(path[i]);
        
       // print_directions();
    }
}


void
draw_start_intersection(unsigned intersection_id) {
    /**
    @param: unique intersection id
    @func: draw the intersection corresponding to @param
     */
    std:: cout<<"before load image\n";
    LatLon intersection_position = getIntersectionPosition(intersection_id);
    std::vector<double>xy = to_cartesian(intersection_position);
    double x = xy[0], y = xy[1];
    std:: cout<<"in load image\n";
    // since (x,y) -> top left corner of png
    // shift Marker20x20.png over 10 pixels left and 20 pixels up
    // so bottom of Marker points to POI
    t_point world_point(x, y);
    t_point screen_coords = world_to_scrn(world_point);
    t_point new_screen_coords(screen_coords.x - 25 , screen_coords.y - 47);
    t_point new_world_coords = scrn_to_world(new_screen_coords);
    draw_surface(start_marker, new_world_coords.x, new_world_coords.y);
}

void
draw_end_intersection(unsigned intersection_id) {
    /**
    @param: unique intersection id
    @func: draw the intersection corresponding to @param
     */
    std:: cout<<"before load image\n";
    LatLon intersection_position = getIntersectionPosition(intersection_id);
    std::vector<double>xy = to_cartesian(intersection_position);
    double x = xy[0], y = xy[1];
    std:: cout<<"in load image\n";
    // since (x,y) -> top left corner of png
    // shift Marker20x20.png over 10 pixels left and 20 pixels up
    // so bottom of Marker points to POI
    t_point world_point(x, y);
    t_point screen_coords = world_to_scrn(world_point);
    t_point new_screen_coords(screen_coords.x - 25 , screen_coords.y - 47);
    t_point new_world_coords = scrn_to_world(new_screen_coords);
    draw_surface(end_marker, new_world_coords.x, new_world_coords.y);
}

void
act_on_find_path_button_func(void (*drawscreen_ptr) (void)) {
    /**
    @param: void (*drawscreen_ptr) (void)
    @func: Gets user input and zooms to the intersection
     */

    // Callback function for the new button we created. This function will be called
    // when the user clicks on the button. It just counts how many
    // times you have clicked the button.  
    poi_path = true;
    std::cout << "poi set to true\n";
    /*
    unsigned intersect1, intersect2;
    double turn_penalty;
    std::string street1_intersection1, street2_intersection1;
    std::string street1_intersection2, street2_intersection2;
    std::cout << "Please type in the first street name for the start intersection: \n";
    std::getline(std::cin, street1_intersection1);
    std::cout << "Please type in the second street name for the start intersection: \n";
    std::getline(std::cin, street2_intersection1);
    std::cout << "Please type in the first street name for the end intersection: \n";
    std::getline(std::cin, street1_intersection2);
    std::cout << "Please type in the second street name for the end intersection: \n";
    std::getline(std::cin, street2_intersection2);
    std::cout << "Please type in turn penalty \n";
    std::cin >> turn_penalty;
    path_find = true;
    // find the start intersection id and end intersection id
    std::vector<unsigned> start_intersection = find_intersection_ids_from_street_names(street1_intersection1, street2_intersection1);
    std::vector<unsigned> end_intersection = find_intersection_ids_from_street_names(street1_intersection2, street2_intersection2);
    //check if there is just one path
    if (start_intersection.size() == 1 && end_intersection.size() == 1) {
        shortest_path = find_path_between_intersections(start_intersection[0], end_intersection[0], turn_penalty);
        std::cout << "1" << std::endl;
        //highlight_street(shortest_path);
        drawscreen();
    } else if (start_intersection.size() == 0)
        std::cout << "Error: No start intersection found \n";
    else if (end_intersection.size() == 0)
        std::cout << "Error: No end intersection found \n";
    else
        std::cout << "Error: multiple path because of more than 2 intersections \n";
     */
}


void print_directions() {

    t_bound_box current_box = get_visible_world();

    //Calculates direction dimensions on current view
    double width = current_box.right() - current_box.left();
    double height = current_box.top() - current_box.bottom();

    double xcenter = current_box.left() + width / 5;
    double ycenter = current_box.bottom() + height * 0.9;

    t_point center(xcenter, ycenter);

    setcolor(BLUE);
    setfontsize(12);
    drawtext(center, "DIRECTIONS:", current_box);
    
    center.offset(0, -height / 30);
    
    std::string current_street;
    std::string previous_street = "";
    
    double street_distance = 0;
    double street_segment_distance = 0;
    
    unsigned shortest_path_size = shortest_path.size();
    
    //Loops through all the street segments in the shortest_path vector
    for (unsigned i = 0; i < shortest_path_size; i++) {

        current_street = street_segment_id_to_street_name[shortest_path[i]];
        
        std::vector<double> curve_point_lengths = street_segment_id_to_curve_point_lengths[shortest_path[i]];
        
        //Finds distance of the current street segment
        for (unsigned j = 0; j < curve_point_lengths.size(); j++){
            street_segment_distance += curve_point_lengths[j];
        }
        street_distance += street_segment_distance;

        //If we are now traveling on a new street
        if (current_street != previous_street && current_street != "<unknown>") {
            drawtext(center, "Turn and continue on " + current_street + " for " + std::to_string((int)round(street_distance)) + " meter(s)", current_box);
            center.offset(0, -height / 30);
            
            street_distance = 0;
        }
        
        street_segment_distance = 0;
        previous_street = current_street;
    }
    
}

void
act_on_help_button_func(void (*drawscreen_ptr) (void)) {

    is_help = true;
    drawscreen();
    is_help = false;
}

//Draws an intersection and its information at the location

void
draw_intersection_info(unsigned intersection_id) {
    /**
    @param: unsigned intersection_id
    @func: Gets information from location vector and draws at location
     */

    /*** Make white box ***/
    std::vector<double> intersection_position = intersection_id_to_cartesian_point[click_intersection_id];
    std::vector<std::string> street_names;
    std::string intersection_name = getIntersectionName(intersection_id);
    street_names = find_intersection_street_names(click_intersection_id);
    std::sort(street_names.begin(), street_names.end());
    street_names.erase(std::unique(street_names.begin(), street_names.end()), street_names.end());
    t_bound_box current(get_visible_world());
    float width = current.get_width();
    float height = current.get_height();

    //make the white windowintersecetion_position.x
    setcolor(WHITE);
    fillrect(intersection_position[0], intersection_position[1], intersection_position[0] + width / 4, intersection_position[1] + height / 6);
    //print the info for the intersection
    setcolor(BLACK);
    setfontsize(10);
    t_point intersecetion_position(intersection_position [0], intersection_position[1]);
    t_point center = t_point(intersecetion_position.x + width / 20, intersecetion_position.y + height / 7);
    drawtext(center, "Intersection Name :", current);
    center = t_point(intersecetion_position.x + width / 8, intersecetion_position.y + height / 9);
    drawtext(center, intersection_name, current);
    center = t_point(intersecetion_position.x + width / 26.5, intersecetion_position.y + height / 9 - (height / 7 - height / 9));
    drawtext(center, "Street Name :\n", current);
    for (unsigned i = 0; i < street_names.size(); ++i) {
        std::string street_name_print = street_names[i];
        center = t_point(intersecetion_position.x + width / 8, intersecetion_position.y + height / 9 - (i + 2)*(height / 7 - height / 9));
        drawtext(center, street_names[i], current);
    }


}

void draw_POI_info(unsigned click_POI_id) {

    std::string POI_name = getPointOfInterestName(click_POI_id);
    std::string POI_type = getPointOfInterestType(click_POI_id);
    LatLon POI_position = getPointOfInterestPosition(click_POI_id);
    std::vector<double>xy = to_cartesian(POI_position);
    double x = xy[0], y = xy[1];
    t_bound_box current(get_visible_world());
    float width = current.get_width();
    float height = current.get_height();
    //make the white windowintersecetion_position.x
    setcolor(WHITE);
    fillrect(x, y - height / 48, x + width / 6, y + height / 48);
    //print the info for the intersection
    setcolor(BLACK);
    setfontsize(10);
    //t_point xy(x,y);
    t_point center = t_point(x + width / 12, y + height / 148);
    drawtext(center, POI_name, current);
    center = t_point(x + width / 12, y - height / 148);
    drawtext(center, "(" + POI_type + ")", current);

}


// create the find button and functionality

void
act_on_find_button_func(void (*drawscreen_ptr) (void)) {
    /**
    @param: void (*drawscreen_ptr) (void)
    @func: Gets user input and zooms to the intersection
     */

    // Callback function for the new button we created. This function will be called
    // when the user clicks on the button. It just counts how many
    // times you have clicked the button.  
    std::vector<unsigned> intersections;
    std::string street_name1, street_name2;
    std::cout << "Please type in the first street name: \n";
    std::getline(std::cin, street_name1);
    std::cout << "Please type in the second street name: \n";
    std::getline(std::cin, street_name2);
    intersections = find_intersection_ids_from_street_names(street_name1, street_name2);

    if (intersections.size() > 0) {
        g_find_intersection = true;
        for (unsigned i = 0; i < intersections.size(); ++i) {
            std::cout << intersections[i] << std::endl;
            std::vector<double> xy1 = intersection_id_to_cartesian_point[intersections[i]];
            double x = xy1[0], y = xy1[1];
            double off = 0.00007;
            //center=t_point(xy1[0], xy1[1]);t_point
            //set_drawing_buffer(OFF_SCREEN);
            intersection_id_found.push_back(intersections[i]);
            // draw_intersection(intersections[i]);
            // copy_off_screen_buffer_to_screen();

            set_visible_world(x - off, y - off, x + off, y + off);
            //drawscreen_ptr();
            //std::cout << "1\n";
        }
        drawscreen();
    } else
        g_find_intersection = false;
}

// create the switch map button and functionality

void
act_on_switch_button_func(void (*drawscreen_ptr) (void)) {
    /**
    @param: void (*drawscreen_ptr) (void)
    @func: Frees all memory, closes map and loads the next map
     */

    //delete data2;
    delete g_IntersectionTree;
    delete g_POITree;
    delete g_PolygonTree;
    delete g_StreetSegmentTree;
    delete segment_id_to_street_type;
    delete OSMID_way_map;
    delete feature_areas;
    delete feature_lengths;
    delete polygons;

    clearscreen();
    closeOSMDatabase();
    close_map(); // calls closeStreetDatabase()

    // Callback function for the new button we created. This func is called
    // when the user clicks on the button.  
    --current_map_index;

    // if no more maps to load -- close the graphics window
    if (current_map_index < 0)
        current_map_index = map_paths.size() - 1;

    std::string map_path = map_paths[current_map_index];

    //Load the map and related data structures
    bool load_success = load_map(map_path);

    if (!load_success) {
        std::cerr << "Failed to load map '" << map_path << "'\n";

        return;
    }

    std::cout << "Successfully loaded map '" << map_path << "'\n";

    draw_map();
}

//Clears the screen and uses double buffering to draw all map details and labels

void
drawscreen(void) // screen redraw
{
    /**
    @param:  
    @func: redraw the screen (e.g. all streets, street names, features, etc.)
     */
    // currently visible box -- how zoomed in is the user
    t_bound_box current_box = get_visible_world();
    double current_area_visible = current_box.area();
    double zoom_level = 100 * current_area_visible / original_world_area;
    std::cout << zoom_level << "\n";

    // draw everything to buffer
    set_drawing_buffer(OFF_SCREEN);

    // Erase old graphics
    clearscreen();



    /*** DRAW FEATURES ***/
    //set_drawing_buffer(OFF_SCREEN);
    draw_all_features_in_box(current_box);
    //copy_off_screen_buffer_to_screen();


    /*** DRAW STREETS ***/
    //set_drawing_buffer(OFF_SCREEN);
    draw_all_streets_in_box(current_box);
    //copy_off_screen_buffer_to_screen();

    /*** DRAW ONE WAY MARKERS ***/
    //    set_drawing_buffer(OFF_SCREEN);
    //    draw_all_one_way_markers(zoom_level);
    //    copy_off_screen_buffer_to_screen();

    /*** DRAW INTERSECTION(S) -> FIND BUTTON ***/
    //set_drawing_buffer(OFF_SCREEN);
    if (g_find_intersection) {
        for (unsigned i = 0; i < intersection_id_found.size(); ++i)
            highlight_intersection(intersection_id_found[i]);
    }
    //copy_off_screen_buffer_to_screen();





    /*** DRAW STREETNAMES ***/
    //set_drawing_buffer(OFF_SCREEN);
    draw_all_street_names(zoom_level);
    //copy_off_screen_buffer_to_screen();

    /*** DRAW INTERSECTIONS ***/
    //set_drawing_buffer(OFF_SCREEN);
    //draw_all_intersections_in_box(current_box);
    //copy_off_screen_buffer_to_screen();


    /*** DRAW POI ***/
    //set_drawing_buffer(OFF_SCREEN);
    draw_all_POI_in_box(current_box);
    //copy_off_screen_buffer_to_screen();

    /*** DRAW POI NAMES***/
    //set_drawing_buffer(OFF_SCREEN);
    //draw_all_POI_names(zoom_level);
    //copy_off_screen_buffer_to_screen();

    /*** DRAW INTERSECTION(S) -> Click intersection ***/
    //set_drawing_buffer(OFF_SCREEN);
    if (click_intersection) {
        highlight_intersection(click_intersection_id);
        draw_intersection_info(click_intersection_id);
    }
    //copy_off_screen_buffer_to_screen();

    /*** DRAW INTERSECTION(S) -> Click intersection by right button***/
    if (click_intersection_right) {
        std::cout << "Error: in click \n";
        highlight_street(shortest_path);
        
        //highlight_intersection(click_intersection_id);
        draw_start_intersection(click_intersection_id);
        //draw_intersection(click_intersection_id);
        //highlight_intersection(click_intersection_id_right);
        draw_end_intersection(click_intersection_id_right);
    }

    /*** HIGHLIGHT SHORTEST PATH ***/
    if (path_find) {
        std::cout << "Error: in path_find \n";
      //  draw_start_intersection(first_inter_id);
        draw_start_intersection(first_inter_id);
      //  highlight_intersection(first_inter_id);
      //  highlight_intersection(second_inter_id);
        draw_end_intersection(second_inter_id);
        std::cout << "didadi" << std::endl;
        highlight_street(shortest_path);
      //  path_find = false;
        ready_to_search = false;
        poi_path = false;
        std::cout << "end draw path" << std::endl;
    }


    /*** DRAW POI -> Click POI ***/
    //set_drawing_buffer(OFF_SCREEN);
   // if (click_POI) {
   //     draw_POI_info(click_POI_id);
   // }


    /*** DRAW SEARCH BAR ***/
    // search_bar->draw_search_bar();
    if (searchbar) {
        draw_search_bar();
    }
    /*** DRAW SEARCH BAR TEXT ***/
    //draw_search_bar_text();

    /*** SHOW PATH ***/
    if (ready_to_search) {
        show_path();
        path_find = false;
        ready_to_search = false;
    }

    
    /*** KEEP PATH ***/
    
     highlight_street(shortest_path);
    
    
    
    //If Help button was pressed
    if (is_help) {
        //Calculates help menu dimensions based on current view
        double width = current_box.right() - current_box.left();
        double height = current_box.top() - current_box.bottom();

        double xcenter = current_box.left() + width / 20;
        double ycenter = current_box.bottom() + height * 0.9;

        t_point center(xcenter, ycenter);

        setcolor(RED);
        setfontsize(10);
        drawtext(center, "Click on the search bar", current_box);

        center.offset(0, -height / 30);
        drawtext(center, "above and enter the ", current_box);

        center.offset(0, -height / 30);
        drawtext(center, "street names to specify", current_box);

        center.offset(0, -height / 30);
        drawtext(center, "the intersections", current_box);

        center.offset(0, -height / 30);
        drawtext(center, "for pathfinding.", current_box);

        //Right instructions
        xcenter = current_box.right() - width / 18;
        ycenter = current_box.bottom() + height * 0.7;

        t_point center2(xcenter, ycenter);

        drawtext(center2, "Click \"Find\" and enter", current_box);

        center2.offset(0, -height / 30);
        drawtext(center2, "street names to display", current_box);

        center2.offset(0, -height / 30);
        drawtext(center2, "information on intersection.", current_box);

        center2.offset(0, -height / 10);
        drawtext(center2, "Left click on intersection", current_box);

        center2.offset(0, -height / 30);
        drawtext(center2, "to select it, and right click", current_box);

        center2.offset(0, -height / 30);
        drawtext(center2, "on another one to display", current_box);

        center2.offset(0, -height / 30);
        drawtext(center2, "the fastest path between", current_box);

        center2.offset(0, -height / 30);
        drawtext(center2, "the two points on the map.", current_box);
    }
    copy_off_screen_buffer_to_screen();
}

//Shows additional information when user clicks on a map location

void
act_on_mouse_button(float x, float y, t_event_buttonPressed buttonPressed) {
    /**
    @param: float x, float y, t_event_buttonPressed buttonPressed
    @func: Gets cursor mouse input and displays intersection info
     */

    // the point click is in the area for search bar
    // if()
    // {
    //     searchbar=true;
    // }





    if (buttonPressed.button == 1) {
        std::cout << "Mouse press at (" << x << ", " << y << ")\n";

        if (buttonPressed.ctrl_pressed)
            std::cout << "Ctrl was held down\n";

        //t_point click = t_point(x, y);
        //t_point click_in_world = scrn_to_world(click);
        //std::cout << "Mouse press at (" << click_in_world .x << ", " << click_in_world .y << ")\n";
        LatLon click_point = to_LatLon(x, y);

        // if this point is not far from the click
        click_intersection_id = find_closest_intersection(click_point);
        click_POI_id = find_closest_point_of_interest(click_point);

        std::cout << "CLICK INTERSECTION ID" << click_intersection_id << std::endl;
        std::cout << "CLICK POI ID" << click_intersection_id << std::endl;

        std::vector<double> intersection_position = intersection_id_to_cartesian_point[click_intersection_id];
        std::vector<double> POI_position = to_cartesian(getPointOfInterestPosition(click_POI_id));

        //convert world to screen
        t_point click_in_screen = world_to_scrn(t_point(x, y));

        t_point intersection_in_screen = world_to_scrn(t_point(intersection_position[0], intersection_position[1]));
        t_point POI_in_screen = world_to_scrn(t_point(POI_position[0], POI_position[1]));

        double intersection_distance = (click_in_screen.x - intersection_in_screen.x) * (click_in_screen.x - intersection_in_screen.x)
                + (click_in_screen.y - intersection_in_screen.y) * (click_in_screen.y - intersection_in_screen.y);
        double POI_distance = (click_in_screen.x - POI_in_screen.x) * (click_in_screen.x - POI_in_screen.x)
                + (click_in_screen.y - POI_in_screen.y) * (click_in_screen.y - POI_in_screen.y);

        //LatLon intersection_ = to_LatLon(intersection_position[0], intersection_position[1]);
        //double distance = find_distance_between_two_points(click_point, intersection_);
        std::cout << intersection_distance << std::endl;

        // if (!path_find) {
        /*** CLICK - INTERSECTION ***/
        if (intersection_distance < 300) {
            click_intersection_right = false;
            click_intersection = true;
            /*  std::vector<std::string> street_names;
              street_names = find_intersection_street_names(click_intersection_id);
              std::sort(street_names.begin(), street_names.end());
              street_names.erase(std::unique(street_names.begin(), street_names.end()), street_names.end());
              // print the street names of this intersection
              for (unsigned i = 0; i < street_names.size(); ++i)
                  std::cout << street_names[i] << std::endl;
              drawscreen();
              draw_intersection_info(click_intersection_id);*/
        } else
            click_intersection = false;

        /*** CLICK - POI ***/
        if (POI_distance < 400) {
            click_intersection_right = false;
            click_POI = true;
            std::string POI_name = getPointOfInterestName(click_POI_id);
            std::cout << POI_name << '\n';
        } else
            click_POI = false;
    }

    if (click_intersection || click_POI) drawscreen();

    /*** FIND PATH ---- CLICK - INTERSECTION ***/
    if (buttonPressed.button == 3) {
        click_intersection = false;
        click_intersection_right = true;
        path_find = true;
        std::cout << "Mouse press at (" << x << ", " << y << ")\n";
        LatLon click_point = to_LatLon(x, y);

        click_intersection_id_right = find_closest_intersection(click_point);
        std::cout << "CLICK INTERSECTION ID" << click_intersection_id_right << std::endl;
        std::vector<double> intersection_position = intersection_id_to_cartesian_point[click_intersection_id_right];

        shortest_path = find_path_between_intersections(click_intersection_id, click_intersection_id_right, 0);
        std::cout << "1" << std::endl;

        // t_color highway_colour(255, 255, 153, 255);
        //  std::cout << "Error: multiple path \n";
        /*  for (unsigned i = 0; i < shortest_path.size(); i++) {
              setcolor(highway_colour);
              draw_street_segment(shortest_path[i]);
          } */

    }

    if (click_intersection_right || path_find) drawscreen();

}

//-----------------------------------//
//-----DRAWING STREETS FUNCTIONS-----//
//-----------------------------------//

//Draws curved street segments by connecting curve points

void
draw_curve(unsigned street_segment_id) {
    /**
    @param: unsigned street_segment_id
    @func: Gets curve point data from vectors and draws lines connecting them
     */

    std::vector<LatLon> curve_points = street_segment_id_to_curve_points[street_segment_id];
    StreetSegmentInfo segment_info = getStreetSegmentInfo(street_segment_id);
    unsigned num_of_curve_points = segment_info.curvePointCount;

    for (unsigned j = 0; j < num_of_curve_points; ++j) {
        if (j == 0) {
            std::vector<double> xy1 = to_cartesian(getIntersectionPosition(segment_info.from));
            std::vector<double> xy2 = to_cartesian(curve_points[j]);

            drawline(xy1[0], xy1[1], xy2[0], xy2[1]);
        } else {

            std::vector<double> xy1 = to_cartesian(curve_points[j]);
            std::vector<double> xy2 = to_cartesian(curve_points[j - 1]);

            drawline(xy1[0], xy1[1], xy2[0], xy2[1]);
        }
    }
    std::vector<double> xy1 = to_cartesian(curve_points[num_of_curve_points - 1]);
    std::vector<double> xy2 = to_cartesian(getIntersectionPosition(segment_info.to));

    drawline(xy1[0], xy1[1], xy2[0], xy2[1]);
}

//Implements street segment rtree to determine if segment is wihin current view
//Draws segment only if within view and at suitable zoom level

void
draw_all_streets_in_box(t_bound_box t_box) {
    /**
    @param: t_bound_box t_box
    @func: Implements street segment rtree to determine if segment is wihin current view
     * Draws segment only if within view and at suitable zoom level
     */

    //Initialize colour for major roads

    t_color highway_colour(255, 255, 153, 255);

    t_point bottom_left = t_box.bottom_left();
    t_point top_right = t_box.top_right();

    double current_area_visible = t_box.area();
    double zoom_level = 100 * current_area_visible / original_world_area;

    std::vector<segmentValue> result;

    double dx = 2 * abs(top_right.x - bottom_left.x);
    double dy = 2 * abs(top_right.y - bottom_left.y);

    // get all intersections in box with leeway
    box2 query_box(point2(bottom_left.x - dx, bottom_left.y - dy), point2(top_right.x + dx, top_right.y + dy));
    g_StreetSegmentTree->query(bgi::covered_by(query_box), std::back_inserter(result));

    // draw each street segment

    BOOST_FOREACH(segmentValue const& v, result) {
        /*
        "motorway"
        "trunk"
        "primary"
        "secondary"
        "tertiary"
        "residential"
      
        "motorway_link"
        "trunk_link" 
        "primary_link" 
        "secondary_link" 
        "tertiary_link"
        "unknown"
         */
        if (zoom_level > 80) {
            if ((*segment_id_to_street_type)[v.second] == "motorway" ||
                    (*segment_id_to_street_type)[v.second] == "trunk") {
                setlinewidth(3);
                setcolor(highway_colour);
                draw_street_segment(v.second);
            }
        } else if (zoom_level > 20) {
            if ((*segment_id_to_street_type)[v.second] == "motorway" ||
                    (*segment_id_to_street_type)[v.second] == "trunk") {
                setlinewidth(3);
                setcolor(highway_colour);
                draw_street_segment(v.second);
            }
            if ((*segment_id_to_street_type)[v.second] == "primary" ||
                    (*segment_id_to_street_type)[v.second] == "secondary") {
                setlinewidth(2);
                setcolor(WHITE);
                draw_street_segment(v.second);
            }
        } else if (zoom_level > 10) {
            if ((*segment_id_to_street_type)[v.second] == "motorway" ||
                    (*segment_id_to_street_type)[v.second] == "trunk") {
                setlinewidth(3);
                setcolor(highway_colour);
                draw_street_segment(v.second);
            }
            if ((*segment_id_to_street_type)[v.second] == "primary" ||
                    (*segment_id_to_street_type)[v.second] == "secondary") {
                setlinewidth(2);
                setcolor(WHITE);
                draw_street_segment(v.second);
            }
            if ((*segment_id_to_street_type)[v.second] == "tertiary") {
                setlinewidth(1.25);
                setcolor(WHITE);
                draw_street_segment(v.second);
            }
        } else if (zoom_level > 5) {
            if ((*segment_id_to_street_type)[v.second] == "motorway" ||
                    (*segment_id_to_street_type)[v.second] == "trunk") {
                setlinewidth(3);
                setcolor(highway_colour);
                draw_street_segment(v.second);
            }
            if ((*segment_id_to_street_type)[v.second] == "primary" ||
                    (*segment_id_to_street_type)[v.second] == "secondary") {
                setlinewidth(2);
                setcolor(WHITE);
                draw_street_segment(v.second);
            }
            if ((*segment_id_to_street_type)[v.second] == "tertiary") {
                setlinewidth(1.25);
                setcolor(WHITE);
                draw_street_segment(v.second);
            }
        } else if (zoom_level > 2.5) {
            if ((*segment_id_to_street_type)[v.second] == "motorway" ||
                    (*segment_id_to_street_type)[v.second] == "trunk") {
                setlinewidth(6);
                setcolor(highway_colour);
                draw_street_segment(v.second);
            }
            if ((*segment_id_to_street_type)[v.second] == "primary" ||
                    (*segment_id_to_street_type)[v.second] == "secondary") {
                setlinewidth(3);
                setcolor(WHITE);
                draw_street_segment(v.second);
            }
        } else if (zoom_level > 1.25) {
            if ((*segment_id_to_street_type)[v.second] == "motorway" ||
                    (*segment_id_to_street_type)[v.second] == "trunk") {
                setlinewidth(6);
                setcolor(highway_colour);
                draw_street_segment(v.second);
            }
            if ((*segment_id_to_street_type)[v.second] == "primary" ||
                    (*segment_id_to_street_type)[v.second] == "secondary") {
                setlinewidth(3.5);
                setcolor(WHITE);
                draw_street_segment(v.second);
            }
            if ((*segment_id_to_street_type)[v.second] == "tertiary" ||
                    (*segment_id_to_street_type)[v.second] == "residential" ||
                    (*segment_id_to_street_type)[v.second] == "motorway_link" ||
                    (*segment_id_to_street_type)[v.second] == "trunk_link" ||
                    (*segment_id_to_street_type)[v.second] == "primary_link" ||
                    (*segment_id_to_street_type)[v.second] == "secondary_link" ||
                    (*segment_id_to_street_type)[v.second] == "tertiary_link") {
                setlinewidth(2);
                setcolor(WHITE);
                draw_street_segment(v.second);
            }
        } else if (zoom_level > 0.625) {
            if ((*segment_id_to_street_type)[v.second] == "motorway" ||
                    (*segment_id_to_street_type)[v.second] == "trunk") {
                setlinewidth(8);
                setcolor(highway_colour);
                draw_street_segment(v.second);
            }
            if ((*segment_id_to_street_type)[v.second] == "primary" ||
                    (*segment_id_to_street_type)[v.second] == "secondary") {
                setlinewidth(5);
                setcolor(WHITE);
                draw_street_segment(v.second);
            }
            if ((*segment_id_to_street_type)[v.second] == "tertiary" ||
                    (*segment_id_to_street_type)[v.second] == "residential" ||
                    (*segment_id_to_street_type)[v.second] == "motorway_link" ||
                    (*segment_id_to_street_type)[v.second] == "trunk_link" ||
                    (*segment_id_to_street_type)[v.second] == "primary_link" ||
                    (*segment_id_to_street_type)[v.second] == "secondary_link" ||
                    (*segment_id_to_street_type)[v.second] == "tertiary_link") {
                setlinewidth(2.5);
                setcolor(WHITE);
                draw_street_segment(v.second);
            }
        } else if (zoom_level > 0.3125) {
            if ((*segment_id_to_street_type)[v.second] == "motorway" ||
                    (*segment_id_to_street_type)[v.second] == "trunk") {
                setlinewidth(8);
                setcolor(highway_colour);
                draw_street_segment(v.second);
            }
            if ((*segment_id_to_street_type)[v.second] == "primary" ||
                    (*segment_id_to_street_type)[v.second] == "secondary") {
                setlinewidth(5);
                setcolor(WHITE);
                draw_street_segment(v.second);
            }
            if ((*segment_id_to_street_type)[v.second] == "tertiary" ||
                    (*segment_id_to_street_type)[v.second] == "residential" ||
                    (*segment_id_to_street_type)[v.second] == "motorway_link" ||
                    (*segment_id_to_street_type)[v.second] == "trunk_link" ||
                    (*segment_id_to_street_type)[v.second] == "primary_link" ||
                    (*segment_id_to_street_type)[v.second] == "secondary_link" ||
                    (*segment_id_to_street_type)[v.second] == "tertiary_link") {
                setlinewidth(2.5);
                setcolor(WHITE);
                draw_street_segment(v.second);
            }
        } else {
            if ((*segment_id_to_street_type)[v.second] == "motorway" ||
                    (*segment_id_to_street_type)[v.second] == "trunk") {
                setlinewidth(10);
                setcolor(highway_colour);
                draw_street_segment(v.second);
            }
            if ((*segment_id_to_street_type)[v.second] == "primary" ||
                    (*segment_id_to_street_type)[v.second] == "secondary") {
                setlinewidth(7);
                setcolor(WHITE);
                draw_street_segment(v.second);
            } else {

                setlinewidth(4);
                setcolor(WHITE);
                draw_street_segment(v.second);
            }
        }

    }

}

//Draws street segment

void
draw_street_segment(unsigned street_segment_id) {
    /**
    @param: unsigned street_segment_id
    @func: Gets segment info and draws the segment
     */

    //Gets info about specific street segment
    StreetSegmentInfo segment_info = getStreetSegmentInfo(street_segment_id);
    unsigned num_of_curve_points = segment_info.curvePointCount;

    //Gets cartesian coordinates of start and end points of segment
    std::vector<double> xy1 = intersection_id_to_cartesian_point[segment_info.from];
    std::vector<double> xy2 = intersection_id_to_cartesian_point[segment_info.to];

    if (num_of_curve_points == 0)
        drawline(xy1[0], xy1[1], xy2[0], xy2[1]);

    else
        draw_curve(street_segment_id);
}

//Draws street name at by calling draw_street_name()

void
draw_all_street_names(double zoom_level) {
    /**
    @param: double zoom_level
    @func: Draws street name
     */

    if (zoom_level < 0.2) {
        setcolor(BLACK);
        setfontsize(8);
        unsigned num_seg = getNumberOfStreetSegments();

        for (unsigned i = 0; i < num_seg; ++i) {
            StreetSegmentInfo segment_info = getStreetSegmentInfo(i);
            std::string street_name = getStreetName(segment_info.streetID);
            if (street_name != "<unknown>") {
                std::vector<double> xy1 = intersection_id_to_cartesian_point[segment_info.from];
                std::vector<double> xy2 = intersection_id_to_cartesian_point[segment_info.to];

                //Computes angle of rotation for text to set text rotation
                double dx = xy2[0] - xy1[0];
                double dy = xy2[1] - xy1[1];
                double angle = atan(dy / dx) / DEG_TO_RAD;

                settextrotation(angle);

                t_point from_bound(xy1[0] - 1, xy1[1]);
                t_point to_bound(xy2[0] + 1, xy2[1]);
                t_bound_box segment_box(from_bound, to_bound);

                if (segment_info.oneWay) {
                    street_name.append("     →");
                    drawtext_in(segment_box, street_name, 0.000005 * zoom_level);
                } else
                    drawtext_in(segment_box, street_name, 0.000005 * zoom_level);
            }
            settextrotation(0);
        }
    } else if (zoom_level < 1) {
        setcolor(BLACK);
        setfontsize(10);
        unsigned num_seg = getNumberOfStreetSegments();

        for (unsigned i = 0; i < num_seg; ++i) {
            StreetSegmentInfo segment_info = getStreetSegmentInfo(i);
            std::string street_name = getStreetName(segment_info.streetID);
            if (street_name != "<unknown>") {
                std::vector<double> xy1 = intersection_id_to_cartesian_point[segment_info.from];
                std::vector<double> xy2 = intersection_id_to_cartesian_point[segment_info.to];

                //Computes angle of rotation for text to set text rotation
                double dx = xy2[0] - xy1[0];
                double dy = xy2[1] - xy1[1];
                double angle = atan(dy / dx) / DEG_TO_RAD;

                settextrotation(angle);

                t_point from_bound(xy1[0], xy1[1]);
                t_point to_bound(xy2[0], xy2[1]);
                t_bound_box segment_box(from_bound, to_bound);

                if (segment_info.oneWay) {
                    street_name.append("     →");
                    drawtext_in(segment_box, street_name);
                } else
                    drawtext_in(segment_box, street_name);
            }
            settextrotation(0);
        }
    } else if (zoom_level < 5) {
        setcolor(BLACK);
        setfontsize(6);
        unsigned num_seg = getNumberOfStreetSegments();

        for (unsigned i = 0; i < num_seg; ++i) {
            StreetSegmentInfo segment_info = getStreetSegmentInfo(i);
            std::string street_name = getStreetName(segment_info.streetID);
            if (street_name != "<unknown>") {
                std::vector<double> xy1 = intersection_id_to_cartesian_point[segment_info.from];
                std::vector<double> xy2 = intersection_id_to_cartesian_point[segment_info.to];

                //Computes angle of rotation for text to set text rotation
                double dx = xy2[0] - xy1[0];
                double dy = xy2[1] - xy1[1];
                double angle = atan(dy / dx) / DEG_TO_RAD;

                settextrotation(angle);

                t_point from_bound(xy1[0], xy1[1]);
                t_point to_bound(xy2[0], xy2[1]);
                t_bound_box segment_box(from_bound, to_bound);

                if (segment_info.oneWay) {
                    street_name.append("     →");
                    drawtext_in(segment_box, street_name);
                } else
                    drawtext_in(segment_box, street_name);
            }
            settextrotation(0);
        }
    } else {
        setcolor(BLACK);
        setfontsize(10);
        unsigned num_seg = getNumberOfStreetSegments();
        unsigned counter = 0;
        for (unsigned i = 0; i < num_seg; ++i) {
            StreetSegmentInfo segment_info = getStreetSegmentInfo(i);
            std::string street_name = getStreetName(segment_info.streetID);
            std::string street_type = (*segment_id_to_street_type)[i];

            if ((street_type == "motorway" ||
                    street_type == "trunk") && street_name != "<unknown>") {

                if (counter % 5 == 0) {

                    std::vector<double> xy1 = intersection_id_to_cartesian_point[segment_info.from];
                    std::vector<double> xy2 = intersection_id_to_cartesian_point[segment_info.to];

                    //Computes angle of rotation for text to set text rotation
                    double dx = xy2[0] - xy1[0];
                    double dy = xy2[1] - xy1[1];
                    double angle = atan(dy / dx) / DEG_TO_RAD;

                    settextrotation(angle);

                    t_point from_bound(xy1[0], xy1[1]);
                    t_point to_bound(xy2[0], xy2[1]);
                    t_bound_box segment_box(from_bound, to_bound);

                    if (segment_info.oneWay) {
                        street_name.append("     →");
                        drawtext_in(segment_box, street_name, 1);
                    } else
                        drawtext_in(segment_box, street_name, 1);

                    counter++;
                }


            }
        }
    }

    //        for (unsigned i = 0; i < num_street; ++i) {
    //            std::vector<unsigned> segment_of_street = data_->street_id_to_street_segments_map[i];
    //            unsigned median_of_seg = 0.5 * segment_of_street.size();
    //            LatLon from_latlon = getIntersectionPosition(getStreetSegmentInfo(segment_of_street[median_of_seg]).from);
    //            LatLon to_latlon = getIntersectionPosition(getStreetSegmentInfo(segment_of_street[median_of_seg]).to);
    //           std::vector< double >xy1 =to_cartesian(from_latlon );
    //           double x1=xy1[0];
    //           double y1=xy1[1];
    //           std::vector< double >xy2 =to_cartesian(to_latlon );
    //           double x2=xy2[0];
    //           double y2=xy2[1];
    //           std::string street_name=getStreetName(getStreetSegmentInfo(segment_of_street[median_of_seg]).streetID);
    //             settextrotation(atan((y2-y1)/(x2-x1))*180/PI);
    //             drawtext((0.5*(x1+x2)),0.5*(y1+y2),street_name,1);
    //            
    //
    //
    //        }
}

/*void
draw_all_street_names(double zoom_level) {
    setcolor(BLACK);
    unsigned num_streets = getNumberOfStreets();
    std::vector<std::string> street_name_exist;
    bool exist = false;
    for (unsigned i = 0; i < num_streets; ++i) {
        std::string street_name = getStreetName(i);
        if (street_name == "<unknown>") continue;

        std::vector<unsigned> street_segment_ids = data_->street_id_to_street_segments_map[i];
        unsigned num_street_segments = street_segment_ids.size();

        //  unsigned frequency = 1000 * num_street_segments / zoom_level;
        //  if(frequency < 3)  frequency = 3;

        for (unsigned j = 0; j < num_street_segments; ++j) {
            StreetSegmentInfo segment_info = getStreetSegmentInfo(street_segment_ids[j]);
            unsigned speed_limit = segment_info.speedLimit;

            // if (j % frequency == 0) {
            //Gets cartesian coordinates of start and end points of segment 
            std::vector<double> xy1 = data_->intersection_id_to_cartesian_point[segment_info.from];
            std::vector<double> xy2 = data_->intersection_id_to_cartesian_point[segment_info.to];

            //Computes angle of rotation for text to set text rotation
            double dx = xy2[0] - xy1[0];
            double dy = xy2[1] - xy1[1];
            double angle = atan(dy / dx) / DEG_TO_RAD;

            settextrotation(angle);

            t_point from_bound(xy1[0], xy1[1]);
            t_point to_bound(xy2[0], xy2[1]);
            t_bound_box segment_box(from_bound, to_bound);


            //If zoom level is extremely close
            if (zoom_level < 5) {
                if (speed_limit > 60) {
                    setfontsize(10);
                } else if (speed_limit > 25) {
                    setfontsize(7);
                } else {
                    setfontsize(5);
                }
            }//If zoom level is very close
            else if (zoom_level < 20) {

                if (speed_limit > 60) {
                    setfontsize(10);
                } else if (speed_limit > 25) {
                    setfontsize(7);
                } else {
                    continue;
                }

            }//If zoom level is moderate
            else if (zoom_level < 30) {
                continue;
            }//If zoom level is very far
            else {
                continue;
            }
            //            street_name_exist.push_back(street_name);
            //            for (unsigned i = 0; i < street_name_exist.size(); ++i) {
            //                if (street_name_exist[i] == street_name) {
            //                    exist = true;
            //                    break;
            //                }
            //
            //            }
            //      if (exist == false)
            drawtext_in(segment_box, street_name, 0.001);
            //settextrotation(0);
            //  }
        }

    }

    
    unsigned num_of_street_segments = getNumberOfStreetSegments();

    //Loops through all street segments and draws their names
    for (unsigned i = 0; i < num_of_street_segments; ++i) 
    {
        //Gets speed limits for each street segment
        StreetSegmentInfo segment_info = getStreetSegmentInfo(i);
        unsigned speed_limit = getStreetSegmentInfo(i).speedLimit;

        //Gets cartesian coordinates of start and end points of segment
        std::vector<double> xy1 = data_->intersection_id_to_cartesian_point[segment_info.from];
        std::vector<double> xy2 = data_->intersection_id_to_cartesian_point[segment_info.to];

        //
        //
        //
        
        t_point from_bound(xy1[0], xy1[1]);
        t_point   to_bound(xy2[0], xy2[1]);
        t_bound_box segment_box(from_bound, to_bound);
        
        //
        //
        //
        
        //Gets street segment IDs from street ID
        unsigned street_id = segment_info.streetID;
        std::vector<unsigned> street_segments = data_->street_id_to_street_segments_map[street_id];

        //Gets index of street_segment_id in street_segments vector
        std::vector<unsigned>::iterator it = std::find(street_segments.begin(), street_segments.end(), i);
        unsigned index = std::distance(street_segments.begin(), it);

        //Computes angle of rotation for text to set text rotation
        double dx = xy2[0] - xy1[0];
        double dy = xy2[1] - xy1[1];
        double angle = atan(dy / dx) / DEG_TO_RAD;

        settextrotation(angle);

        //Draws street names for periodic street segments
        if (index % 10 == 0)
            draw_street_name(i, xy1, xy2, zoom_level, speed_limit, segment_box);

        settextrotation(0);
    }
 */
//}

//Draws street name at location

void
draw_street_name(unsigned street_segment_id, double zoom_level, unsigned speed_limit, t_bound_box segment_box) {
    /**
    @param: unsigned street_segment_id, double zoom_level, unsigned speed_limit, t_bound_box segment_box
    @func: Gets street string and draws street name
     */

    //Gets street name from vector
    std::string street_name = street_segment_id_to_street_name[street_segment_id];

    //If street segment name is unknown
    if (street_name == "<unknown>")
        return;

    setcolor(BLACK);

    //If zoom level is extremely close
    if (zoom_level < 5) {
        if (speed_limit > 60) {
            setfontsize(10);
        } else if (speed_limit > 25) {
            setfontsize(7);
        } else {
            setfontsize(5);
        }
    }//If zoom level is very close
    else if (zoom_level < 20) {

        if (speed_limit > 60) {
            setfontsize(10);
        } else if (speed_limit > 25) {
            setfontsize(7);
        } else {
            return;
        }

    }//If zoom level is moderate
    else if (zoom_level < 50) {

    }//If zoom level is very far
    else {

        return;
    }

    drawtext_in(segment_box, street_name, 0.01);

    //drawtext((xy1[0] + xy2[0]) / 2, (xy1[1] + xy2[1]) / 2, street_name, speed_limit * 0.000005, speed_limit * 0.000001);
}




//-----------------------------------------//
//-----DRAWING INTERSECTIONS FUNCTIONS-----//
//-----------------------------------------//

//Highlights intersection

void
highlight_intersection(unsigned intersection_id) {

    /**
    @param: unique intersections id
    @func: highlight intersections corresponding to @param
     */
    std::vector<double>xy = intersection_id_to_cartesian_point[intersection_id];
    double x = xy[0], y = xy[1];
    double off = 0.000001;
    setcolor(RED);
    fillrect(x - off, y - off, x + off, y + off);
}

//Draws intersection indication

void
draw_intersection(unsigned intersection_id) {

    /**
    @param: unique intersections id
    @func: draw intersections corresponding to @param
     */
    std::vector<double>xy = intersection_id_to_cartesian_point[intersection_id];
    double x = xy[0], y = xy[1];
    double off = 0.00001;

    setcolor(BLUE);
    fillrect(x, y, x + off, y + off);
}

//Calls draw_intersection() for every intersection

void
draw_all_intersections() {
    /**
    @param: 
    @func: draw all intersections for given map
     */
    unsigned num_intersections = getNumberOfIntersections();

    for (unsigned i = 0; i < num_intersections; ++i)
        draw_intersection(i);
}

//Implements rtree for intersection and determines if it's inside current view
//Calls draw_intersection() if it's within view

void
draw_all_intersections_in_box(t_bound_box t_box) {
    /**
    @param: bounding box (section of graphics window) 
    @func: draw all intersections inside @param
     */
    t_point bottom_left = t_box.bottom_left();
    t_point top_right = t_box.top_right();

    std::vector<value2> result;

    box2 query_box(point2(bottom_left.x, bottom_left.y), point2(top_right.x, top_right.y));

    g_IntersectionTree->query(bgi::covered_by(query_box), std::back_inserter(result));

    for (unsigned i = 0; i < result.size(); ++i)
        draw_intersection(result[i].second);
}





//--------------------------------------------//
//-----DRAWING NATURAL FEATURES FUNCTIONS-----//
//--------------------------------------------//

//Determines the type of feature and draws on screen

void
draw_feature(unsigned feature_id) {
    /**
    @param: unique feature id
    @func: draw the feature corresponding to @param
     */

    std::vector<LatLon> feature_points = std::vector<LatLon>();
    FeatureType feature_name = getFeatureType(feature_id);

    if (feature_name == Lake || feature_name == River || feature_name == Stream) {
        t_color blue_color(102, 178, 255, 255);
        setcolor(blue_color);
    } else if (feature_name == Park || feature_name == Greenspace || feature_name == Golfcourse || feature_name == Island) {
        t_color green_color(102, 255, 102, 255);
        setcolor(green_color);
    } else if (feature_name == Beach) {
        t_color yellow_color(255, 255, 153, 255);
        setcolor(yellow_color);
    } else {
        t_color grey_color(160, 160, 160, 255);
        setcolor(grey_color);
    }

    t_point *points = new t_point[getFeaturePointCount(feature_id)];

    for (unsigned i = 0; i < getFeaturePointCount(feature_id); ++i)
        feature_points.push_back(getFeaturePoint(feature_id, i));

    if ((*feature_areas)[feature_id] != 0) { // closed polygon
        for (unsigned i = 0; i < feature_points.size(); ++i) {
            t_point point_s;
            point_s.x = to_cartesian(feature_points[i])[0];
            point_s.y = to_cartesian(feature_points[i])[1];
            *(points + i) = point_s;
        }
        fillpoly(points, feature_points.size());
    } else {
        for (unsigned i = 0; i < feature_points.size() - 1; ++i) {

            std::vector<double> xy1 = to_cartesian(feature_points[i]);
            std::vector<double> xy2 = to_cartesian(feature_points[i + 1]);
            drawline(xy1[0], xy1[1], xy2[0], xy2[1]);
        }
    }
    delete [] points;
}

//Calls draw_feature() for all features

void
draw_all_features() {
    /**
    @param:  
    @func: draw all features for the given map 
     */
    unsigned num_features = getNumberOfFeatures();

    for (unsigned i = 0; i < num_features; ++i)
        draw_feature(i);
}

//Implements feature rtree and determines if it is within current view
//Calls draw_feature() if within view

void
draw_all_features_in_box(t_bound_box t_box) {
    /**
    @param: bounding box (section of graphics window) 
    @func: draw all features inside @param
     */
    t_point bottom_left = t_box.bottom_left();
    t_point top_right = t_box.top_right();
    double current_area_visible = t_box.area();
    double zoom_level = 100 * current_area_visible / original_world_area;
    std::vector<value_polygon> result_s;

    box2 query_box(point2(bottom_left.x, bottom_left.y), point2(top_right.x, top_right.y));
    g_PolygonTree->query(bgi::intersects(query_box), std::back_inserter(result_s));

    // Vector of vectors that stores feature ID and area
    std::vector<std::pair<unsigned, double>> feature_id_and_area;

    BOOST_FOREACH(value_polygon const& v, result_s) {
        feature_id_and_area.push_back(std::make_pair(v.second, (*feature_areas)[v.second]));
    }

    //Sorts the vector by area
    std::sort(feature_id_and_area.begin(), feature_id_and_area.end(),
            [](const std::pair<unsigned, double>& a, const std::pair<unsigned, double>& b) {
                return a.second > b.second;
            });

    // Vector of vectors that stores feature ID and length
    std::vector<std::pair<unsigned, double>> feature_id_and_length;

    BOOST_FOREACH(value_polygon const& v, result_s) {
        feature_id_and_length.push_back(std::make_pair(v.second, (*feature_lengths)[v.second]));
    }

    //Sorts the vector by length
    std::sort(feature_id_and_length.begin(), feature_id_and_length.end(),
            [](const std::pair<unsigned, double>& a, const std::pair<unsigned, double>& b) {
                return a.second > b.second;
            });

    for (unsigned i = 0; i < feature_id_and_area.size(); ++i) {
        if (zoom_level > 80) {
            if (feature_id_and_area[i].second > feature_areas_mean * 10 ||
                    feature_id_and_length[i].second > feature_lengths_mean * 10)
                draw_feature(feature_id_and_area[i].first);
        } else if (zoom_level > 20) {
            if (feature_id_and_area[i].second > feature_areas_mean * 5 ||
                    feature_id_and_length[i].second > feature_lengths_mean * 5)
                draw_feature(feature_id_and_area[i].first);
        } else if (zoom_level > 10) {
            if (feature_id_and_area[i].second > feature_areas_mean * 2.5 ||
                    feature_id_and_length[i].second > feature_lengths_mean * 2.5)
                draw_feature(feature_id_and_area[i].first);
        } else if (zoom_level > 5) {
            if (feature_id_and_area[i].second > feature_areas_mean * 1.5 ||
                    feature_id_and_length[i].second > feature_lengths_mean * 1.5)
                draw_feature(feature_id_and_area[i].first);
        } else if (zoom_level > 2.5) {
            if (feature_id_and_area[i].second > feature_areas_mean ||
                    feature_id_and_length[i].second > feature_lengths_mean)
                draw_feature(feature_id_and_area[i].first);
        } else if (zoom_level > 1.25) {
            if (feature_id_and_area[i].second > feature_areas_mean * 0.5 ||
                    feature_id_and_length[i].second > feature_lengths_mean * 0.5)
                draw_feature(feature_id_and_area[i].first);
        } else if (zoom_level > 0.625) {
            if (feature_id_and_area[i].second > feature_areas_mean * 0.25 ||
                    feature_id_and_length[i].second > feature_lengths_mean * 0.25)
                draw_feature(feature_id_and_area[i].first);
        } else if (zoom_level > 0.3125) {
            if (feature_id_and_area[i].second > feature_areas_mean * 0.125 ||
                    feature_id_and_length[i].second > feature_lengths_mean * 0.125)
                draw_feature(feature_id_and_area[i].first);
        } else {
            draw_feature(feature_id_and_area[i].first);
        }

    }

}





//--------------------------------------------//
//-----DRAWING NATURAL FEATURES FUNCTIONS-----//
//--------------------------------------------//

//Loads png marker at point of interest

void
draw_POI(unsigned id) {
    /**
    @param: unique POI id
    @func: draw the POI corresponding to @param
     */
    LatLon POI_position = getPointOfInterestPosition(id);
    std::vector<double>xy = to_cartesian(POI_position);
    double x = xy[0], y = xy[1];

    // since (x,y) -> top left corner of png
    // shift Marker20x20.png over 10 pixels left and 20 pixels up
    // so bottom of Marker points to POI
    t_point world_point(x, y);
    t_point screen_coords = world_to_scrn(world_point);
    t_point new_screen_coords(screen_coords.x - 10, screen_coords.y + 20);
    t_point new_world_coords = scrn_to_world(new_screen_coords);

    draw_surface(map_marker, new_world_coords.x, new_world_coords.y);
}

//Calls draw_POI() for every point of interest

void
draw_all_POI() {
    /**
    @param: 
    @func: draws all POI for given map
     */
    unsigned num_of_POI = getNumberOfPointsOfInterest();

    for (unsigned i = 0; i < num_of_POI; ++i)
        draw_POI(i);
}

//Implements POI rtree and determines if it's within current view
//Calls draw_POI() if within view

void
draw_all_POI_in_box(t_bound_box t_box) {
    /**
    @param: bounding box (section of graphics window) 
    @func: draws all POI inside @param
     */
    t_point bottom_left = t_box.bottom_left();
    t_point top_right = t_box.top_right();
    double current_area_visible = t_box.area();

    double zoom_level = 100 * current_area_visible / original_world_area;

    std::vector<value2> result;

    box2 query_box(point2(bottom_left.x, bottom_left.y), point2(top_right.x, top_right.y));

    g_POITree->query(bgi::covered_by(query_box), std::back_inserter(result));

    for (unsigned i = 0; i < result.size(); ++i) {

        if (zoom_level < 0.02)
            draw_POI(result[i].second);
    }
}

//Draws POI name at given location

void
draw_all_POI_names(double zoom_level) {
    /**
    @param: how zoomed in is the user (%)
    @func: draws POI based on @param
     */
    unsigned num_of_POI = getNumberOfPointsOfInterest();

    setcolor(BLACK);

    //Draws all names for each POI
    for (unsigned i = 0; i < num_of_POI; ++i) {
        std::string POI_name = getPointOfInterestName(i);

        //Gets the position of the POI
        LatLon latlon_position = getPointOfInterestPosition(i);
        std::vector <double> POI_position = to_cartesian(latlon_position);

        t_point POI_center(POI_position[0], POI_position[1]);

        //If zoom level is extremely close
        if (zoom_level < 5) {
            setfontsize(10);
            drawtext(POI_center, POI_name, 0.01, 0.01);
        }//If zoom level is very close
        else if (zoom_level < 20) {
            setfontsize(5);
            drawtext(POI_center, POI_name, 0.01, 0.01);

        }//If zoom level is moderate
        else {

            continue;
        }
    }
}



//----------------------------//
//-----HELPER FUNCTION(S)-----//
//----------------------------//

//Converts LatLon point to Cartesian coords
//Returns vector of x, y coords

std::vector<double>
to_cartesian(LatLon point_) {

    /**
    @param: latlon point
    @return: vector of cartesian point ([0]->x, [1]->y)
     */
    double point_lat_rad = point_.lat() * DEG_TO_RAD;
    double point_lon_rad = point_.lon() * DEG_TO_RAD;

    double x = point_lon_rad * cos(average_lat);
    double y = point_lat_rad;

    std::vector<double> cartesian_point = {x, y};

    return cartesian_point;
}

// convert between LatLon and Cartesian (x, y, z)

LatLon
to_LatLon(float x, float y) {
    /**
    @param: cartesian points x, y
    @return: latlon point
     */
    double point_lat = y / DEG_TO_RAD;
    double point_lon_rad = x / cos(average_lat);
    double point_lon = point_lon_rad / DEG_TO_RAD;
    LatLon latlon_point = LatLon(point_lat, point_lon);

    return latlon_point;
}

//Computes area and length of polygon and returns vector of information

std::vector<double>
get_polygon_area_or_length(unsigned feature_id) // [0]->area || [1]->length
{
    /**
    @param: unique feature id
    @return: vector of area and length ([0]->area, [1]->length),
             if closed polygon then length = 0 and area != 0 (vice versa for open polygon)
     */
    std::vector<LatLon> coordinates;
    double area = 0, length = 0;

    for (unsigned i = 0; i < getFeaturePointCount(feature_id); ++i)
        coordinates.push_back(getFeaturePoint(feature_id, i));

    // find area (closed polygon)
    if (coordinates[0].lat() == coordinates[coordinates.size() - 1].lat() &&
            coordinates[0].lon() == coordinates[coordinates.size() - 1].lon()) {
        for (unsigned i = 0; i < coordinates.size() - 1; ++i) {
            area += (to_cartesian(coordinates[i])[0]) * (to_cartesian(coordinates[i + 1])[1])
                    - (to_cartesian(coordinates[i + 1])[0]) * (to_cartesian(coordinates[i])[1]);
        }
        area += (to_cartesian(coordinates[coordinates.size() - 1])[0]) * (to_cartesian(coordinates[0])[1])
                - (to_cartesian(coordinates[0])[0]) * (to_cartesian(coordinates[coordinates.size() - 1])[1]);

        area = abs(area) / 2.0;
    }// find length
    else {
        for (unsigned i = 0; i < coordinates.size() - 1; ++i) {
            std::vector<double> xy1 = to_cartesian(coordinates[i]);
            std::vector<double> xy2 = to_cartesian(coordinates[i + 1]);

            length += sqrt((xy2[0] - xy1[0]) * (xy2[0] - xy1[0]) + (xy2[1] - xy1[1]) * (xy2[1] - xy1[1]));
        }
    }

    std::vector<double> area_length;
    area_length.push_back(area);
    area_length.push_back(length);

    return area_length;
}

//Fills OSMID information map

void
fill_OSMID_way_map() {
    /**
    @param:
    @func: fill OSMID_way_map with unsigned corresponding to a OSMWay
     */
    for (unsigned i = 0; i < getNumberOfWays(); ++i) {

        const OSMEntity* osm_entity = getWayByIndex(i);
        OSMID temp_osmid = osm_entity->id();
        (*OSMID_way_map)[temp_osmid] = i;
    }
}

//Retrieves type of street from OSM data
//Returns string indicating street type

std::string
type_of_highway_street(unsigned street_segment_id) {
    /**
    @param: unique street segment id
    @return: the type of "highway" street @param corresponds to
     */
    StreetSegmentInfo segment_info = getStreetSegmentInfo(street_segment_id);
    OSMID OSM_info = segment_info.wayOSMID;
    std::string highway_street;
    unsigned index = (*OSMID_way_map)[OSM_info];
    const OSMEntity* osm_entity = getWayByIndex(index);

    for (unsigned i = 0; i < getTagCount(osm_entity); ++i) {
        std::pair<std::string, std::string> osm_pair = getTagPair(osm_entity, i);

        if (osm_pair.first == "highway")
            highway_street = osm_pair.second;
    }
    return highway_street;
}

//Inserts street type into vector as string

void
insert_streets_into_vectors() {
    /**
    @param:
    @func: insert the street type (string) into the segment_id_to_street_type vector
     */
    for (unsigned i = 0; i < getNumberOfStreetSegments(); ++i) {
        std::string highway_type = type_of_highway_street(i);
        (*segment_id_to_street_type)[i] = highway_type;
    }
}
