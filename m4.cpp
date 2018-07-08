#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "m4.h"

#include "Global.h"
#include "IntersectionNode.h"

#include <cmath>
#include <algorithm>
#include <vector>
#include <iterator>
#include <queue>

#include <utility>
#include <queue>
#include <iostream>
#include <iterator>     
#include <climits>

#include <boost/timer.hpp>

struct open_set_compare_ {

    bool operator()(const std::pair<double, unsigned>& lhs, const std::pair<double, unsigned>& rhs) const {
        return lhs.first > rhs.first;
    }
};

std::vector<unsigned> traveling_courier(const std::vector<DeliveryInfo>& deliveries,
                                        const std::vector<unsigned>& depots,
                                        const float turn_penalty) 
{
    boost::timer t;

    /*
     movement: i -> j
     i - right
     j - down
    - - - - -  
    |1|2|3|4|
    - - - - - 
    |5|6|7|8| 
    - - - - -
     */

    unsigned deliveries_size = deliveries.size();

    std::vector<std::vector<double>> timeMap(2 * deliveries_size, std::vector<double>(2 * deliveries_size)); // Defaults to zero initial value

    // 1. pickUp - pickUp
    #pragma omp parallel for
    for (unsigned i = 0; i < deliveries_size; ++i) {
        for (unsigned j = 0; j < deliveries_size; ++j) {
            if (i == j)
                timeMap[i][j] = INT_MAX;
            else if (deliveries[i].pickUp == deliveries[j].pickUp)
                timeMap[i][j] = 0;
            else {
                double distance = find_distance_between_two_points(intersection_id_to_position[deliveries[i].pickUp], 
                                                                   intersection_id_to_position[deliveries[j].pickUp]);

                //std::vector<unsigned> path = find_path_between_intersections(deliveries[i].pickUp, deliveries[j].pickUp, turn_penalty);
                double travel_time_ = distance; //compute_path_travel_time(path, turn_penalty);

                //intersectionsToPathVector[i][j] = path;

                //if (!travel_time_)
                //    timeMap[i][j] = INT_MAX;
                //else
                timeMap[i][j] = travel_time_;
            }
        }
    }

    // 2. dropOff - pickUp / 3. pickUp - dropOff
    #pragma omp parallel for
    for (unsigned i = 0; i < deliveries_size; ++i) {
        for (unsigned j = 0; j < deliveries_size; ++j) {
            if (deliveries[i].dropOff == deliveries[j].pickUp)
                timeMap[i + deliveries_size][j] = 0;
            else {
                double distance = find_distance_between_two_points(intersection_id_to_position[deliveries[i].dropOff], 
                                                                   intersection_id_to_position[deliveries[j].pickUp]);

                //std::vector<unsigned> path = find_path_between_intersections(deliveries[i].dropOff, deliveries[j].pickUp, turn_penalty);
                double travel_time_ = distance; //compute_path_travel_time(path, turn_penalty);

                //intersectionsToPathVector[i + deliveries.size()][j] = path;

                //if (!travel_time_)
                //    timeMap[i + deliveries.size()][j] = INT_MAX;
                //else
                // 2.
                timeMap[i + deliveries_size][j] = travel_time_;
                // 3.
                timeMap[j][i + deliveries_size] = travel_time_;
            }
        }
    }

    // 3. pickUp - dropOff
    //#pragma omp parallel for
    //    for (unsigned i = 0; i < deliveries_size; ++i) 
    //    {
    //        for (unsigned j = 0; j < deliveries_size; ++j) 
    //        {
    //            if (deliveries[i].pickUp == deliveries[j].dropOff)
    //                timeMap[i][j + deliveries_size] = 0;
    //            else 
    //            {
    //                double distance = timeMap[j][i+deliveries_size];
    //                //double distance = find_distance_between_two_points(getIntersectionPosition(deliveries[i].pickUp), getIntersectionPosition(deliveries[j].dropOff));
    //                
    //                //std::vector<unsigned> path = find_path_between_intersections(deliveries[i].pickUp, deliveries[j].dropOff, turn_penalty);
    //                double travel_time_ = distance; //compute_path_travel_time(path, turn_penalty);
    //
    //                //intersectionsToPathVector[i][j + deliveries.size()] = path;
    //                
    //                //if (!travel_time_)
    //                //    timeMap[i][j + deliveries.size()] = INT_MAX;
    //                //else
    //                    timeMap[i][j + deliveries_size] = travel_time_;
    //            }
    //        }
    //    }

    // 4. dropOff - dropOff
    #pragma omp parallel for
    for (unsigned i = 0; i < deliveries_size; ++i) {
        for (unsigned j = 0; j < deliveries_size; ++j) {
            if (i == j)
                timeMap[i + deliveries_size][j + deliveries_size] = INT_MAX;
            else if (deliveries[i].dropOff == deliveries[j].dropOff)
                timeMap[i + deliveries_size][j + deliveries_size] = 0;
            else {
                double distance = find_distance_between_two_points(intersection_id_to_position[deliveries[i].dropOff], 
                                                                   intersection_id_to_position[deliveries[j].dropOff]);

                //std::vector<unsigned> path = find_path_between_intersections(deliveries[i].dropOff, deliveries[j].dropOff, turn_penalty);
                double travel_time_ = distance; //compute_path_travel_time(path, turn_penalty);

                //intersectionsToPathVector[i + deliveries.size()][j + deliveries.size()] = path;

                //if (!travel_time_)
                //    timeMap[i + deliveries.size()][j + deliveries.size()] = INT_MAX;
                //else
                timeMap[i + deliveries_size][j + deliveries_size] = travel_time_;
            }
        }
    }
        
    
    std::vector<std::vector<unsigned>> paths;
    std::vector<double> path_times;
    
    
    
    for(unsigned i = 0; i < depots.size() && t.elapsed() < 0.96; ++i)
    {
        
        std::vector<unsigned> intersectionIDs;
        std::vector<unsigned> intersection_index;

        // vector of all the legal pickUps -- @ beginning is full (start at 0)
        std::vector<bool> legalPickUps(deliveries_size, true);

        // vector of all legal dropOffs -- @ beginning empty (start at deliveries.size())
        std::vector<bool> legalDropOffs(2 * deliveries_size, false);

        /************************************************/
        /*************** GREEDY ALGORITHM ***************/
        /************************************************/

        //------------------------- 1. START AT RANDOM DEPOT -------------------------//



        // start at random depot
        //unsigned start_depot = depots[rand() % depots.size()];
        unsigned start_depot = depots[i];


        intersectionIDs.push_back(start_depot);
        intersection_index.push_back(start_depot);

        // first closest pickUp i index
        unsigned first_pickup_index;

        // go from RANDOM DEPOT to CLOSEST PICKUP
        std::vector<unsigned> path = find_path_from_depot_to_first_pickUp(deliveries, start_depot, first_pickup_index, turn_penalty);

        intersection_index.push_back(first_pickup_index);

        // set current pickUp to the closest initial pickUp (intersection ID)
        unsigned currentPickUp = deliveries[first_pickup_index].pickUp;

        intersectionIDs.push_back(currentPickUp);

        // get all legal drop offs corresponding to currentPickUp    
        legalDropOffs[first_pickup_index + deliveries_size] = true;

        // update current legal pickUps
        legalPickUps[first_pickup_index] = false;

        // @ first pickUp intersection 
        unsigned currentLocation = currentPickUp;
        unsigned currentLocationIndex = first_pickup_index;

        //not only pick up s 
        while (1 /*std::find(legalPickUps.begin(),  legalPickUps.end(),  true) != legalPickUps.end() || 
               std::find(legalDropOffs.begin(), legalDropOffs.end(), true) != legalDropOffs.end()*/) {
            // search for minimum time (j-axis)
            std::vector<double> timesFromCurrentLocation = timeMap[currentLocationIndex];

            unsigned smallestIndex;
            double smallestTime = INT_MAX;

            for (unsigned j = 0; j < timesFromCurrentLocation.size(); ++j) // timesFromCurrentLocation.size() = 2 * deliveries.size()
            {
                if (j < deliveries_size) {
                    if (!legalPickUps[j]) continue; // invalid pickUp
                }
                else {
                    if (!legalDropOffs[j]) continue; // invalid dropOff
                }

                if (timesFromCurrentLocation[j] < smallestTime) {
                    smallestTime = timesFromCurrentLocation[j];
                    smallestIndex = j;
                }
            }

            if (smallestTime == INT_MAX) break;

            //                                    //
            // GOTO smallestIndex (smallest time) //
            //                                    //          

            // 1. goto dropOff
            if (smallestIndex >= deliveries_size) {
                unsigned drop_off_location = deliveries[smallestIndex % deliveries_size].dropOff;

                std::vector<unsigned> vec1 = find_path_between_intersections(currentLocation, drop_off_location, turn_penalty);

                if (vec1.size()) path.insert(path.end(), vec1.begin(), vec1.end());

                intersectionIDs.push_back(deliveries[smallestIndex % deliveries.size()].dropOff);

                // update legalDropOffs
                legalDropOffs[smallestIndex] = false;

                intersection_index.push_back(smallestIndex);
                currentLocationIndex = smallestIndex;
                currentLocation = drop_off_location;
            }
                // 2. goto pickUp
            else {
                unsigned pick_up_location = deliveries[smallestIndex].pickUp;

                std::vector<unsigned> vec1 = find_path_between_intersections(currentLocation, pick_up_location, turn_penalty);

                if (vec1.size()) path.insert(path.end(), vec1.begin(), vec1.end());

                intersectionIDs.push_back(deliveries[smallestIndex].pickUp);

                // update legalPickUps
                legalPickUps[smallestIndex] = false;

                // get all legal drop offs corresponding to currentPickUp

                // update legalDropOffs
                legalDropOffs[smallestIndex + deliveries_size] = true;

                intersection_index.push_back(smallestIndex);
                currentLocationIndex = smallestIndex;
                currentLocation = pick_up_location;
            }
        }

        unsigned intersection_id_end;

        std::vector<unsigned> path_to_depot = find_path_from_last_dropOff_to_depots(depots, currentLocation, turn_penalty, intersection_id_end);
        path.insert(path.end(), path_to_depot.begin(), path_to_depot.end());

        intersectionIDs.push_back(intersection_id_end);
        intersection_index.push_back(intersection_id_end);

        //
        //
        //
        //
        //

        paths.push_back(path);
        path_times.push_back(compute_path_travel_time(path, turn_penalty));

    }

    auto it = std::min_element(path_times.begin(), path_times.end());

    for (unsigned i = 0; i < path_times.size(); ++i)
        if (path_times[i] == *it)
            return paths[i];

    //double elapsed_time = t.elapsed();

    //double time_e;
    //time_e = 


    //return path;


    //------------------------------------------------------------------------//

    //----- 2. dropOffsFromPickUps & pickUpsFromDropOffs -----//


    //    for (unsigned i = 0; i < dropOffsFromPickUps)
    //
    //        // search for minimum time (j-axis)
    //        for (unsigned i = 0; i < timesFromCurrentPickUp.size(); ++i)


    //------------------------------------------------------------------------//

    //----- 2. dropOffsFromPickUps & pickUpsFromDropOffs -----//
    /*
            std::unordered_map<unsigned, std::vector<unsigned>> dropOffsFromPickUps; // pickUpIntersectionID  -> <dropOffIntersectionID>
    std::unordered_map<unsigned, std::vector<unsigned>> pickUpsFromDropOffs; // dropOffIntersectionID -> <pickUpIntersectionID>
    std::unordered_map<unsigned, std::vector<unsigned>> corresponding;
    for (unsigned i = 0; i < deliveries.size(); ++i) {
        dropOffsFromPickUps[deliveries[i].pickUp].push_back(deliveries[i].dropOff);
        pickUpsFromDropOffs[deliveries[i].dropOff].push_back(deliveries[i].pickUp);
        //all_pick_up.push_back(deliveries[i].pickUp);
    }
     */

    //    for (unsigned i = 0; i < 2*deliveries.size(); ++i) {
    //        corresponding[deliveries[i].pickUp].push_back(deliveries[i].dropOff);
    //        corresponding[deliveries[i].dropOff].push_back(deliveries[i].pickUp);
    //        //all_pick_up.push_back(deliveries[i].pickUp);
    //    }


    //----- 3. delivery pickUp or dropOff ID -> "name" -----//
    /*
        std::unordered_map<unsigned, std::string> deliveriesIDtoName; // delivery pickUp || dropOff ID -> "name"
        std::unordered_map<unsigned, std::string> depotsIDtoName;     // depot ID's -> "name"
 
        for(unsigned i = 0; i < deliveries.size(); ++i) 
        {
            deliveriesIDtoName[deliveries[i].pickUp]  = "pickUp";
            deliveriesIDtoName[deliveries[i].dropOff] = "dropOff";
        }   
    
        for(unsigned i = 0; i < deliveries.size(); ++i)   
            depotsIDtoName[depots[i]] = "depot";
     */
    //----- 4. rtrees -----//
    /*
        rtree dropOffIntersections; // rtree for deliveries (dropOff)
        rtree pickUpIntersections;  // rtree for deliveries (pickUp)
        rtree depotIntersections;   // rtree for depots
    
        for(unsigned i = 0; i < deliveries.size(); ++i) 
        {
            std::vector<double> points_xyz_pickup = to_cartesian_3D(getIntersectionPosition(deliveries[i].pickUp));
            point p1(points_xyz_pickup[0], points_xyz_pickup[1], points_xyz_pickup[2]);

            std::vector<double> points_xyz_dropoff = to_cartesian_3D(getIntersectionPosition(deliveries[i].dropOff));
            point p2(points_xyz_dropoff[0], points_xyz_dropoff[1], points_xyz_dropoff[2]);
        
            pickUpIntersections.insert(std::make_pair(p1,  deliveries[i].pickUp));
            dropOffIntersections.insert(std::make_pair(p2, deliveries[i].dropOff));
        }
        for(unsigned i = 0; i < depots.size(); ++i) 
        {
            std::vector<double> points_xyz = to_cartesian_3D(getIntersectionPosition(depots[i]));
            point p(points_xyz[0], points_xyz[1], points_xyz[2]);

            depotIntersections.insert(std::make_pair(p, depots[i]));
        }
     */
    /**************************************/
    /********** GREEDY ALGORITHM **********/
    /**************************************/

    // start at first depot
    /*   unsigned current_pickUp = depots[0];

       // get closest pickUp intersection to starting depot
       std::vector<value> temp_result;
       std::vector<double> points_xyz_temp = to_cartesian_3D(getIntersectionPosition(current_pickUp));
       point my_pos_point(points_xyz_temp[0], points_xyz_temp[1], points_xyz_temp[2]);
       pickUpIntersections.query(bgi::nearest(my_pos_point, 1), std::back_inserter(temp_result));

       // goto the nearest pickup intersection from first depot
       std::vector<unsigned> path = find_path_between_intersections(depots[0], temp_result[0].second, turn_penalty);

       // if no path exists return' FIGURE THIS OUT LATER
       if(!path.size()) return std::vector<unsigned>();
    
       // remove pickUp intersection from tree
       pickUpIntersections.remove(temp_result[0]);
       current_pickUp = temp_result[0].second; // @ initial pickUp         
    
       while(1)
       {
           // get location(s) of dropOff intersections corresponding to current_pickUp
           std::vector<unsigned> dropOffLocations = dropOffsFromPickUps[current_pickUp];

           // goto each dropOff location
           for(unsigned i = 0; i < dropOffLocations.size(); ++i)
           {
               std::vector<unsigned> vec1 = find_path_between_intersections(current_pickUp, dropOffLocations[i], turn_penalty);
               path.insert(path.end(), vec1.begin(), vec1.end());
               current_pickUp = dropOffLocations[i]; // @ end of loop contains last dropOff location
           }
        
           // search for next closest pickUp location with respect to current location
           std::vector<value>  result;
           std::vector<double> points_xyz = to_cartesian_3D(getIntersectionPosition(current_pickUp));
           point my_position_point (points_xyz[0], points_xyz[1], points_xyz[2]);
           pickUpIntersections.query(bgi::nearest(my_position_point, 1), std::back_inserter(result));
        
           // break if no more pickUp locations
           if(!result.size()) break; 

           // remove pickUp location from rtree
           pickUpIntersections.remove(result[0]);
        
           // goto pickUp spot from current location
           std::vector<unsigned> vec1 = find_path_between_intersections(current_pickUp, result[0].second, turn_penalty);
           path.insert(path.end(), vec1.begin(), vec1.end());

           // update current pickUp location
           current_pickUp = result[0].second;
       }
    
       // return to first depot
       std::vector<unsigned> vec1 = find_path_between_intersections(current_pickUp, depots[0], turn_penalty);
       path.insert(path.end(), vec1.begin(), vec1.end());

       return path;
     */

    /***********************************************************************************************************/
    /***********************************************************************************************************/
    /********************************* NAIVE METHOD TO TEST FUNCTIONALITY **************************************/
    /***********************************************************************************************************/
    /***********************************************************************************************************/
    /*
    std::vector <unsigned> travel_path;
    std::vector <unsigned> pickUp_skipped;
    std::vector <unsigned> dropOff_skipped;

    unsigned last_visited_intersection = depots[0];

    //Loops through every pickup point and goes to first valid one
    for (unsigned i = 0; i < deliveries.size(); i++) 
    {  
        std::vector <unsigned> pickUp_path = find_path_between_intersections(last_visited_intersection, deliveries[i].pickUp, turn_penalty);

        //If path is valid
        if (pickUp_path.size() != 0) {
            copy_vector(pickUp_path, travel_path);
            last_visited_intersection = deliveries[i].pickUp;
        } else {
            //Skips pickup for later visit
            pickUp_skipped.push_back(deliveries[i].pickUp);
        }
    }

    bool is_erased = false;

    //keeps looping through skipped vector until all are visited
    while (pickUp_skipped.size() != 0) 
    {          
        unsigned num_skipped_pickUp = pickUp_skipped.size();

        //Loops through every pickup point that was skipped
        for (unsigned i = 0; i < num_skipped_pickUp; i++) {
            std::vector <unsigned> pickUp_path = find_path_between_intersections(last_visited_intersection, pickUp_skipped[i], turn_penalty);

            //If path is valid
            if (pickUp_path.size() != 0) {
                copy_vector(pickUp_path, travel_path);
                last_visited_intersection = pickUp_skipped[i];
                pickUp_skipped.erase(pickUp_skipped.begin() + i);

                is_erased = true;
            }
        }

        //If no path was found
        if (!is_erased)
            return std::vector <unsigned>();
    }

    //Loops through every dropoff point and goes to first valid one
    for (unsigned i = 0; i < deliveries.size(); i++) {

        std::vector <unsigned> dropOff_path = find_path_between_intersections(last_visited_intersection, deliveries[i].dropOff, turn_penalty);

        //If path is valid
        if (dropOff_path.size() != 0) {
            copy_vector(dropOff_path, travel_path);
            last_visited_intersection = deliveries[i].dropOff;
        } else {
            //Skips dropoff for later visit
            dropOff_skipped.push_back(deliveries[i].dropOff);
        }
    }

    is_erased = false;

    //keeps looping through skipped vector until all are visited
    while (dropOff_skipped.size() != 0) {

        unsigned num_skipped_dropOff = dropOff_skipped.size();

        //Loops through every pickup point that was skipped
        for (unsigned i = 0; i < num_skipped_dropOff; i++) {
            std::vector <unsigned> dropOff_path = find_path_between_intersections(last_visited_intersection, dropOff_skipped[i], turn_penalty);

            //If path is valid
            if (dropOff_path.size() != 0) {
                copy_vector(dropOff_path, travel_path);
                last_visited_intersection = dropOff_skipped[i];
                dropOff_skipped.erase(dropOff_skipped.begin() + i);

                is_erased = true;
            }
        }

        //If no path was found
        if (!is_erased)
            return std::vector <unsigned>();
    }

    std::vector <unsigned> return_path = find_path_between_intersections(last_visited_intersection, depots[0], turn_penalty);
    copy_vector(return_path, travel_path);

    return travel_path;
     */
}



/******************************************************************************/
/******************************************************************************/
/******************************************************************************/



double rand_double_in_range(double min, double max) {
    double d = (double) rand() / RAND_MAX; // [0, 1]
    return min + d * (max - min);
}

double P(double C_delta, double k, double T) {
    return exp(-1 * C_delta / k * T);
}



//void simulated_annealing(std::vector<unsigned> initial_path, double turn_penalty, double high_temperature)
//{
//    std::vector<unsigned> S = initial_path;    
//    double C = compute_path_travel_time(S, turn_penalty); // E.g. travel time
//    double T = high_temperature;                          // big number
//    double k = 1.38064852 * pow(10,-23);
//    //bool solution_changing = true;
//    
//    while( /*solution_changing &&*/ T > 0 ) 
//    {
//        std::vector<unsigned> S_new = two_opt(S, turn_penalty);       // perturb(S)
//        double C_new = compute_path_travel_time(S_new, turn_penalty); // WAS: compute_path_travel_time(S, turn_penalty)
//        double C_delta = C_new - C;
//        
//        if ( (C_new < C) || rand_double_in_range(0, 1) < P(C_delta, k, T) ) // Update solution
//        {
//            S = S_new;   
//            C = C_new;
//        }
//        
//        --T; //T = reduceTemp(T);
//        //solution_changing = is_solution_changing(S, S_new);
//    }
//}



//Pushes vector elements from vector1 to vector2
void copy_vector(const std::vector<unsigned>& vector1, std::vector<unsigned>& vector2) {
    unsigned vector1_size = vector1.size();

    //Loops through all elements in vector1 and puts them in vector2
    for (unsigned i = 0; i < vector1_size; ++i)
        vector2.push_back(vector1[i]);
}


// if keys.size()>4, use two opt
/*
std::vector<unsigned> two_opt(const std::vector<DeliveryInfo>& deliveries, const float turn_penalty,
        std::vector <int>& keys, std::vector<unsigned>& intersectionIDs) {
    unsigned size = keys.size();
   // unsigned improve = 0;
    bool test_path_legal = false;
    std::vector<unsigned> test_keys;
    // generate two random number
    while (!test_path_legal) {
        bool tested = false;
        test_path_legal = true;
        unsigned first_random = (rand() % (size - 2) )+ 1;
        unsigned second_random = (rand() % (size - 2) )+ 1;
        while (first_random == second_random){
            second_random = (rand() % (size - 2)) + 1;
        }
        // make sure second_random always > first random
        if (second_random < first_random) {
            unsigned temp = second_random;
            second_random = first_random;
            first_random = temp;
        }
        
        test_keys = two_opt_swap(first_random, second_random, keys);
        for (unsigned i = first_random; i < = second_random; i++) {
            for (unsigned j = first_random; j < = second_random; j++) {
                if (keys[i] == keys[j] + deliveries.size()) {
                    test_path_legal = false;
                    tested = true;
                    break;
                }

            }
            if (tested) break;
        }
      
    }
    keys = test_keys;
    std::vector<unsigned> intersection_id_new;
     intersection_id_new.push_back(keys[0]);
    for (unsigned i = 1; i < keys.size()-1; i++) {
        if (keys[i] < deliveries.size())
            intersection_id_new.push_back(deliveries[keys[i]].pickUp);
        else
            intersection_id_new.push_back(deliveries[keys[i] - deliveries.size()].dropOff);
    }
    intersection_id_new.push_back(keys[keys.size()-1]);
    intersectionIDs = intersection_id_new;
    return test_keys;
}
//     std::vector<unsigned> test_keys = two_opt_swap(first_random, second_random, keys);
// if it is legal, construct the path

        for(unsigned i=0; i< test_keys.size(); ++i){
            std::vector<unsigned> path_segment = find_path_between_intersections(test_keys[i],test_keys[i+1],turn_penalty) ;
            temp_path.insert(temp_path.end(), path_segment.begin(), path_segment.end());
        }
        double test_time = compute_path_travel_time(temp_path, turn_penalty);
        if (test_time < time ) {
            time = test_time;
            shortest_path = temp_path;
        }
 
*/



std::vector<unsigned> two_opt_swap(unsigned first_num, unsigned second_num, std::vector <int> keys) {
    unsigned size = keys.size();
    std::vector<unsigned> new_key;

    // 1. take route[0] to route[i-1] and add them in order to new_route
    for (unsigned a = 0; a <= first_num - 1; ++a) {
        new_key.push_back(keys[a]);

    }
    // 2. take route[i] to route[k] and add them in reverse order to new_route
    for (unsigned a = second_num; a >= first_num; a--) {
        new_key.push_back(keys[a]);
    }

    // 3. take route[k+1] to end and add them in order to new_route
    for (int a = second_num + 1; a < size; ++a) {
        new_key.push_back(keys[a]);
    }
    // right now have a new vector of keys

}



std::vector<unsigned>
find_path_from_depot_to_first_pickUp(const std::vector<DeliveryInfo>& deliveries,
        const unsigned depot,
        unsigned &first_pickup_index,
        const double turn_penalty) {

    // priority queue to store nodes that are being preprocessed (openSet)
    std::priority_queue<std::pair<double, unsigned>, std::vector<std::pair<double, unsigned>>, open_set_compare_> openSet;

    // closed set (set of already evaluated nodes)  
    std::vector<bool> closedSet(getNumberOfIntersections(), false);

    // "distance" vector contains times for each node 
    std::vector<double> dist(getNumberOfIntersections(), INT_MAX);

    // Insert starting node in priority queue and initialize its time to 0
    openSet.push(std::make_pair(0.0, depot));
    dist[depot] = 0.0;

    // best previous street seg
    std::vector<unsigned> optimal_prior_streetseg(getNumberOfIntersections(), 0);

    // best previous node
    std::vector<unsigned> best_prior_node(getNumberOfIntersections(), 0);

    bool found = false;
    unsigned intersection_end_id;

    //unsigned first_pick_up_index;
    // set of nodeID's (all keys from cameFromStreetSegment[])
    std::vector<bool> keys(getNumberOfIntersections(), false);

    /***********************************************************************************/
    /* Looping until priority queue becomes empty (or all distances are not finalized) */
    /***********************************************************************************/
    while (!openSet.empty() && !found) {
        // Get top value (min distance) from set
        unsigned current = openSet.top().second;

        // if the current node == destination then break
        //reaching the intersection of POI
        for (unsigned i = 0; i < deliveries.size(); i++) {
            if (current == deliveries[i].pickUp) {
                found = true;
                first_pickup_index = i;
                intersection_end_id = deliveries[i].pickUp;
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

        if (current == depot) {
            for (unsigned i = 0; i < neighbor_nodes.size(); ++i) {
                // Get IntersectionNode object (nodeID, travelTImes, segmentIDs)
                IntersectionNode n = neighbor_nodes[i];
                std::vector<double> weights = n.travelTimes;
                std::vector<unsigned> segments = n.streetSegments;

                for (unsigned j = 0; j < weights.size(); ++j) {
                    // If there is shorter path to n through u.
                    if (dist[n.nodeID] > (dist[current] + weights[j])) {
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
        } else {
            for (unsigned i = 0; i < neighbor_nodes.size(); ++i) {
                // Get IntersectionNode object (nodeID, travelTImes, segmentIDs)
                IntersectionNode n = neighbor_nodes[i];
                std::vector<double> weights = n.travelTimes;
                std::vector<unsigned> segments = n.streetSegments;

                unsigned previousSeg = optimal_prior_streetseg[current];

                for (unsigned j = 0; j < weights.size(); ++j) {
                    // ADD TURN PENALTY TO WEIGHTS
                    if (street_segment_id_to_street_id[previousSeg] != street_segment_id_to_street_id[segments[j]])
                        weights[j] += turn_penalty;

                    // If there is shorter path to n through u.
                    if (dist[n.nodeID] > (dist[current] + weights[j])) {
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

    unsigned key = intersection_end_id;

    std::vector<unsigned> totalSegments;
    totalSegments.push_back(optimal_prior_streetseg[key]);

    while (keys[key]) {
        key = best_prior_node[key];
        if (key != depot) totalSegments.insert(totalSegments.begin(), optimal_prior_streetseg[key]);
    }
    /*assume that there is a way from depot to the first pick up all the time*/
    // if(key != depot) return std::vector<unsigned>();
    return totalSegments;
}



std::vector<unsigned>
find_path_from_last_dropOff_to_depots(const std::vector<unsigned>& depots,
        unsigned end_dropOff,
        const double turn_penalty,
        unsigned &intersection_end_id) {

    // priority queue to store nodes that are being preprocessed (openSet)
    std::priority_queue<std::pair<double, unsigned>, std::vector<std::pair<double, unsigned>>, open_set_compare_> openSet;

    // closed set (set of already evaluated nodes)  
    std::vector<bool> closedSet(getNumberOfIntersections(), false);

    // "distance" vector contains times for each node 
    std::vector<double> dist(getNumberOfIntersections(), INT_MAX);

    // Insert starting node in priority queue and initialize its time to 0
    openSet.push(std::make_pair(0.0, end_dropOff));
    dist[end_dropOff] = 0.0;

    // best previous street seg
    std::vector<unsigned> optimal_prior_streetseg(getNumberOfIntersections(), 0);

    // best previous node
    std::vector<unsigned> best_prior_node(getNumberOfIntersections(), 0);

    bool found = false;
    //unsigned intersection_end_id;
    //unsigned first_pick_up_index;
    // set of nodeID's (all keys from cameFromStreetSegment[])
    std::vector<bool> keys(getNumberOfIntersections(), false);

    /***********************************************************************************/
    /* Looping until priority queue becomes empty (or all distances are not finalized) */
    /***********************************************************************************/
    while (!openSet.empty()&& !found) {
        // Get top value (min distance) from set
        unsigned current = openSet.top().second;

        // if the current node == destination then break
        //reaching the intersection of POI
        for (unsigned i = 0; i < depots.size(); i++) {
            if (current == depots[i]) {
                found = true;
                intersection_end_id = depots[i];
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

        if (current == end_dropOff) {
            for (unsigned i = 0; i < neighbor_nodes.size(); ++i) {
                // Get IntersectionNode object (nodeID, travelTImes, segmentIDs)
                IntersectionNode n = neighbor_nodes[i];
                std::vector<double> weights = n.travelTimes;
                std::vector<unsigned> segments = n.streetSegments;

                for (unsigned j = 0; j < weights.size(); ++j) {
                    // If there is shorter path to n through u.
                    if (dist[n.nodeID] > (dist[current] + weights[j])) {
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
        } else {
            for (unsigned i = 0; i < neighbor_nodes.size(); ++i) {
                // Get IntersectionNode object (nodeID, travelTImes, segmentIDs)
                IntersectionNode n = neighbor_nodes[i];
                std::vector<double> weights = n.travelTimes;
                std::vector<unsigned> segments = n.streetSegments;

                unsigned previousSeg = optimal_prior_streetseg[current];

                for (unsigned j = 0; j < weights.size(); ++j) {
                    // ADD TURN PENALTY TO WEIGHTS
                    if (street_segment_id_to_street_id[previousSeg] != street_segment_id_to_street_id[segments[j]])
                        weights[j] += turn_penalty;

                    // If there is shorter path to n through u.
                    if (dist[n.nodeID] > (dist[current] + weights[j])) {
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

    unsigned key = intersection_end_id;

    std::vector<unsigned> totalSegments;
    totalSegments.push_back(optimal_prior_streetseg[key]);

    while (keys[key]) {
        key = best_prior_node[key];
        if (key != end_dropOff) totalSegments.insert(totalSegments.begin(), optimal_prior_streetseg[key]);
    }
    /*assume that there is a way from depot to the first pick up all the time*/
    // if(key != depot) return std::vector<unsigned>();

    return totalSegments;
}
