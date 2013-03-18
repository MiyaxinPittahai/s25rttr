// Copyright (c) 2005 - 2013 Settlers Freaks (sf-team at siedler25.org)
// Copyright (c) 2013 S25RTTR-Aux/Nevik Rehnel (hai.kataker at gmx.de)
//
// This file is part of Return To The Roots.
//
// Return To The Roots is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 2 of the License, or
// (at your option) any later version.
//
// Return To The Roots is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Return To The Roots. If not, see <http://www.gnu.org/licenses/>.

///////////////////////////////////////////////////////////////////////////////
// Headers
#include "main.h"
#include "Node.h"

#include "GameWorld.h"
#include "noRoadNode.h"
#include "VideoDriverWrapper.h"
#include "Random.h"
#include "MapGeometry.h"
#include "nobHarborBuilding.h"
#include "GameClient.h"

#include <set>
#include <vector>
#include <limits>

#include <iostream>

///////////////////////////////////////////////////////////////////////////////
// Macros / Defines
#if defined _WIN32 && defined _DEBUG && defined _MSC_VER
#   define new new(_NORMAL_BLOCK, THIS_FILE, __LINE__)
#   undef THIS_FILE
    static char THIS_FILE[] = __FILE__;
#endif //_WIN32 && _DEBUG && _MSC_VER

//maximum map size; used for dimensioning the array of nodes on map
#define MAX_MAP_SIZE 1024U

/**
 * Constant value for an invalid predecessor node
 */
const unsigned INVALID_PRED = 0xFFFFFFFF;

/**
 * Comparator for the priority queue or std::set for pathfinding on roads
 */
class RoadNodeComparatorInv {
public:
    /**
     * Comparison operator for pathfinding on roads, works exactly like free pathfinding
     * except we compare noRoadNodes instead of Points
     *
     * @param[in] rn1 one road node
     * @param[in] rn2 another road node
     *
     * @return `true` if rn1's `estimate` is higher than rn2's; if the two estimates are equal,
     *     returns `true` if rn1's coordinate id is higher than rn2's; `false` otherwise
     */
    bool operator()(const noRoadNode* const rn1, const noRoadNode* const rn2) const {
        if (rn1->estimate == rn2->estimate) {
            // If the path cost is the same for both operands, we compare the coordinates (we need
            //   a strictly isotonic sequence for std::set)
            return (rn1->coord_id > rn2->coord_id);
        }

        return (rn1->estimate > rn2->estimate);
    }
};

// "Forward" declaration for PathfindingPoint (implementation follows below) because NewNode and
// PathfindingPoint cross-reference each other
struct PathfindingPoint;

/**
 * @brief Class for one Node and its relevant information
 *
 * We simply save the whole map (see variable 'pf_nodes' below) and thus won't have to allocate
 * and free memory all the time.<br>
 * Nodes in the array have an unambiguous ID (id = y*map_width+x)
 */
struct NewNode {
    NewNode() : way_length(0), count_nodes(0), dir(0), pred(INVALID_PRED), lastVisited(0) {}

    /** path cost for the part of the path from starting node to this node [TODO: rename this] */
    unsigned int way_length;
    /**
     * Number of nodes between starting node and this node;
     * may differ from 'way' when doing pathfinding on roads
     */
    unsigned int count_nodes;
    /** Direction from which this node was reached */
    unsigned char dir;
    /** ID ( = y*map_width+x) of the predecessor node */
    unsigned int pred;
    /** Iterator to position in priority queue (std::set), free pathfinding */
    std::set<PathfindingPoint>::iterator it_p;
    /** (for A* algorithm:) Node was already visited if lastVisited == currentVisit */
    unsigned int lastVisited; // [TODO: check if names lastVisited, currentVisit make sense; change if not]
};

/** Save nodes of the map, assume maximum map size */
NewNode pf_nodes[MAX_MAP_SIZE*MAX_MAP_SIZE];
unsigned int currentVisit = 0;

/**
 * Points as references to the Nodes mentioned above, so we only have to keep the coordinates x,y
 * around in the set
 */
struct PathfindingPoint {
public:
    /**
     * Destination X coordinate for the current pathfinding instance, used for valuation of the
     * point
     */
    static MapCoord dst_x;
    /**
     * Destination Y coordinate for the current pathfinding instance, used for valuation of the
     * point
     */
    static MapCoord dst_y;
    /** Pointer to GameWorld, used to get the IDs and map size */
    static const GameWorldBase * gwb;

    /** This point's X coordinate */
    MapCoord x;
    /** This point's Y coordinate */
    MapCoord y;
    /** This point's coord ID */
    unsigned int id;
    /** Direct distance from this point to the destination point */
    unsigned int distance;

public:
    /**
     * Constructor
     *
     * @param sx the X coordinate of this point
     * @param sy the Y coordinate of this point
     * @param sid the coord ID of this point
     */
    PathfindingPoint(const MapCoord sx, const MapCoord sy, const unsigned int sid) {
        x = sx;
        y = sy;
        id = sid;
        distance = gwb->CalcDistance(x,y,dst_x,dst_y);
    }

    /**
     * Initialize the static variables when beginning a new pathfinding instance
     *
     * @param dst_x destination X coordinate for this pathfinding instance
     * @param dst_y destination Y coordinate for this pathfinding instance
     * @param gwb the GameWorldBase instance that will be used to calculate distances and retrieve
     *      the map size and coord IDs
     */
    static void Init(const MapCoord dst_x, const MapCoord dst_y, const GameWorldBase * gwb) {
        PathfindingPoint::dst_x = dst_x;
        PathfindingPoint::dst_y = dst_y;
        PathfindingPoint::gwb = gwb;
    }

    /**
     * Comparison operator
     *
     * @param otherPPoint
     * @return `true` if the estimated total cost for the path through this point is lower than
     *     through `otherPPoint`; if both estimates are equal, `true` if this point's ID is lower
     *     than `otherPPoint`'s; `false` otherwise
     */
    bool operator<(const PathfindingPoint otherPPoint) const {
        // Estimate path lengths for both points by adding the path length until here and the
        // remaining air-line distance to the destination (a common heuristic for A*)
        unsigned int total_length1 = pf_nodes[id].way_length + distance;
        unsigned int total_length2 = pf_nodes[otherPPoint.id].way_length + otherPPoint.distance;

        // If the path cost is the same for both operands, we compare the coordinates (we need
        // a strictly isotonic sequence for std::set)
        if(total_length1 == total_length2)
            return (id < otherPPoint.id);
        else
            return (total_length1 < total_length2);
    }
};

//[question: why not initialize these when declaring them? (above)]
// Declarations: see above
MapCoord PathfindingPoint::dst_x = 0;
MapCoord PathfindingPoint::dst_y = 0;
const GameWorldBase * PathfindingPoint::gwb = NULL;

/**
 * Pathfinding with A* (runtime O(v*log(v)) on free terrain (not bound to roads)
 *
 * @param x_start
 * @param y_start
 * @param x_dest
 * @param y_dest
 * @param random_route
 * @param max_route
 * @param route
 * @param length
 * @param first_dir
 * @param IsNodeOK
 * @param IsNodeToDestOk
 * @param param
 * @param record
 * @return
 */
// Breaking usual convention for function parameter format here because there's so many
bool GameWorldBase::FindFreePath(
        const MapCoord x_start,
        const MapCoord y_start,
        const MapCoord x_dest,
        const MapCoord y_dest,
        const bool random_route,            //[TODO: add some docs for the params]
        const unsigned max_route,
        std::vector<unsigned char> * route,
        unsigned *length,
        unsigned char * first_dir,
        FP_Node_OK_Callback IsNodeOK,
        FP_Node_OK_Callback IsNodeToDestOk,
        const void * param,
        const bool record) const
{
    // increase currentVisit, so we don't have to clear the visited-states at every run
    currentVisit++;

    // if the counter reaches its maxium, tidy up
    if (currentVisit == std::numeric_limits<unsigned>::max() - 1) {
        for (unsigned i = 0; i < (MAX_MAP_SIZE*MAX_MAP_SIZE); ++i) {
            pf_nodes[i].lastVisited = 0;
        }
        currentVisit = 1;
    }

    std::set<PathfindingPoint> todo;
    PathfindingPoint::Init(x_dest,y_dest,this);

    // insert starting node
    unsigned start_id = MakeCoordID(x_start, y_start);
    std::pair< std::set<PathfindingPoint>::iterator, bool > ret =
            todo.insert(PathfindingPoint(x_start, y_start, start_id));
    // and set values accordingly
    pf_nodes[start_id].it_p = ret.first;
    pf_nodes[start_id].pred = INVALID_PRED;
    pf_nodes[start_id].lastVisited = currentVisit;
    pf_nodes[start_id].way_length = 0;
    pf_nodes[start_id].dir = 0;

    while (todo.size()) { //while there are still nodes we haven't checked yet
        // pick node with lowest path cost (best node)
        PathfindingPoint best = *todo.begin();
        // node is now being checked, remove from to-do list
        todo.erase(todo.begin());

        // get ID of best node
        unsigned best_id = best.id;

        // this node has been removed from the set, so we set its Iterator to the end
        // (i.e. not defined), as somewhat of a "NULL" replacement
        pf_nodes[best_id].it_p = todo.end();

        // have we reached the destination? (prevent zero-path [start==end] by
        // demanding way_length!=0)
        if (x_dest == best.x && y_dest == best.y && pf_nodes[best_id].way_length) {
            // destination reached!
            // return the relevant result data (by pointer)
            if(length)
                *length = pf_nodes[best_id].way_length;
            if(route)
                route->resize(pf_nodes[best_id].way_length);

            // reconstruct route; if desired, save initial direction
            for (unsigned z = pf_nodes[best_id].way_length-1; best_id != start_id;
                    --z, best_id = pf_nodes[best_id].pred) {
                if(route)
                    route->at(z) = pf_nodes[best_id].dir;
                if(first_dir && z == 0)
                    *first_dir = pf_nodes[best_id].dir;
            }

            // done, a path was found
            return true;
        }

        // have we reached max path length? if so, no need to expand this node further
        if(pf_nodes[best_id].way_length == max_route)
            continue;

        // start with random direction if desired (so we don't always go the same way, especially
        // important for soldiers)
        unsigned start = random_route ? RANDOM.Rand("pf",__LINE__,y_start*GetWidth()+x_start,6) : 0;

        // expand nodes into all 6 directions
        for (unsigned z = start; z < start+6; ++z) {
            unsigned i = (z+3)%6;

            // calculate coordinates of this adjacent node
            MapCoord    xa = GetXA(best.x,best.y,i),
                        ya = GetYA(best.x,best.y,i);

            // get ID of the adjacent node
            unsigned xaid = MakeCoordID(xa, ya);

            // have we already checked that node?
            if (pf_nodes[xaid].lastVisited == currentVisit) {
                // in that case, if the path is short from this node to that neighbor than from its
                // saved predecessor, update path length and predecessor
                if (pf_nodes[xaid].it_p != todo.end() &&
                        pf_nodes[best_id].way_length+1 < pf_nodes[xaid].way_length) {
                    pf_nodes[xaid].way_length  = pf_nodes[best_id].way_length+1;
                    pf_nodes[xaid].pred = best_id;
                    todo.erase(pf_nodes[xaid].it_p);
                    ret = todo.insert(PathfindingPoint(xa, ya, xaid));
                    pf_nodes[xaid].it_p = ret.first;
                    pf_nodes[xaid].dir = i;
                }
                // we don't want to check this same node again, so continue with next
                continue;
            }

            // if this isn't the destination yet, check only the normal conditions
            if (!(xa == x_dest && ya == y_dest) && IsNodeOK) {
                if (!IsNodeOK(*this,xa,ya,i,param))
                    continue;
            }

            // if this neighbor is indeed the destination, check additional destination conditions
            if (IsNodeToDestOk) {
                if (!IsNodeToDestOk(*this,xa,ya,i,param))
                    continue;
            }

            // everything is fine, expand this node (set its values, add to to-do list)
            pf_nodes[xaid].lastVisited = currentVisit;
            pf_nodes[xaid].way_length = pf_nodes[best_id].way_length+1;
            pf_nodes[xaid].dir = i;
            pf_nodes[xaid].pred = best_id;

            ret = todo.insert(PathfindingPoint(xa,ya, xaid));
            pf_nodes[xaid].it_p = ret.first;
        }
    }

    // done, but to-do list is empty (no more available node) and no path to destination was found
    //   --> no path exists
    return false;
}

// Code style rules are broken in this class to improve readability of the repeated use of the
// typed priority queue
/**
 * Specialized `std::priority_queue` to manage an ordered list of nodes which are yet to be
 * checked in the A* algorithm for paths on roads (`GameWorldBase::FindPathOnRoads`)
 */
template<
        class _Ty,
        class _Container    = std::vector<_Ty>,
        class _Pr           = std::less<typename _Container::value_type> >
class openlist_container : public   std::priority_queue<_Ty, _Container, _Pr> {
public:
    openlist_container():           std::priority_queue<_Ty, _Container, _Pr>() {
                                    std::priority_queue<_Ty, _Container, _Pr>::c.reserve(255);
    }

    void rearrange(const _Ty& target) {
        typename std::vector<_Ty>::iterator it =
                         std::find( std::priority_queue<_Ty, _Container, _Pr>::c.begin(),
                                    std::priority_queue<_Ty, _Container, _Pr>::c.end(),
                                    target);
        std::push_heap(             std::priority_queue<_Ty, _Container, _Pr>::c.begin(), it+1,
                                    std::priority_queue<_Ty, _Container, _Pr>::comp);
    }

    void clear() {
                                    std::priority_queue<_Ty, _Container, _Pr>::c.clear();
    }
};

/** The strictly ordered list of nodes yet to be checked out */
openlist_container<const noRoadNode*, std::vector<const noRoadNode*>, RoadNodeComparatorInv> todo;

/**
 * Pathfinding with A* (runtime O(v*log(v)) on roads
 *
 * @param start
 * @param goal
 * @param ware_mode
 * @param length
 * @param first_dir
 * @param next_harbor
 * @param forbidden
 * @param record
 * @param max
 * @return
 */
bool GameWorldBase::FindPathOnRoads(
        const noRoadNode * const start,
        const noRoadNode * const goal,
        const bool ware_mode,
        unsigned * length,
        unsigned char * first_dir,
        Point<MapCoord> * next_harbor,
        const RoadSegment * const forbidden,
        const bool record,
        unsigned max) const
{
    // read from replay?
    if (GameClient::inst().ArePathfindingResultsAvailable() && record) {
        unsigned char dir;
        if (GameClient::inst().ReadPathfindingResult(&dir, length, next_harbor)) {
            if(first_dir) *first_dir = dir;
            return (dir != 0xff);
        }
    }

    // if start or goal node are NULL, we cannot find a route
    if (!start || !goal) {
        if (record)
            GameClient::inst().AddPathfindingResult(0xff, length, next_harbor);
        return false;
    }

    // increase currentVisit, so we don't have to clear the visited-states at every run
    currentVisit++;

    // if the counter reaches its maxium, tidy up
    if (currentVisit == std::numeric_limits<unsigned>::max() - 1) {
        for (unsigned i = 0; i < (MAX_MAP_SIZE*MAX_MAP_SIZE); ++i) {
            pf_nodes[i].lastVisited = 0;
        }
        currentVisit = 1;
    }

    // initialize destination data in struct
    PathfindingPoint::Init(goal->GetX(), goal->GetY(), this);

    // clear to-do list
    todo.clear();

    // insert start node into to-do list
    unsigned start_id = start->coord_id;
    start->distance = start->estimate = CalcDistance(start->GetX(), start->GetY(),
        PathfindingPoint::dst_x, PathfindingPoint::dst_y);
    todo.push(start);

    // and set its values accordingly
    pf_nodes[start_id].pred = INVALID_PRED;
    pf_nodes[start_id].lastVisited = currentVisit;
    pf_nodes[start_id].way_length = 0;
    pf_nodes[start_id].count_nodes = 0;
    pf_nodes[start_id].dir = 0;

    while (todo.size()) {
        // select node with lowest path cost (best node)
        const noRoadNode *best = todo.top();

        // node is now being checked, remove from list
        todo.pop();

        // get ID of best node
        unsigned best_id = best->coord_id;

        // have we reached the destination? (prevent zero-path [start==end] by
        // demanding way_length!=0)
        if (best == goal &&  pf_nodes[best_id].way_length) {
            // destination reached!
            unsigned char first_dir_tmp = 0xff;

            // return length if desired (pointer was passed)
            if (length)
                *length = pf_nodes[best_id].way_length;

            // reconstruct route
            for (unsigned z = pf_nodes[best_id].count_nodes-1; best_id != start_id;
                    --z, best_id = pf_nodes[best_id].pred) {
                if (z == 0) {
                    first_dir_tmp = pf_nodes[best_id].dir;

                    if (next_harbor) {
                        next_harbor->x = best_id%width;
                        next_harbor->y = best_id/width;
                    }
                }

            }

            // if desired, return first direction
            if (first_dir)
                *first_dir = first_dir_tmp;

            // done, a path was found
            if (record)
                GameClient::inst().AddPathfindingResult(first_dir_tmp, length, next_harbor);

            return true;
        }

        // follow adjacent flag or roads in all 6 directions
        for (unsigned int i = 0; i < 6; ++i) {
            // check if there is a road or flag
            noRoadNode *rna = best->GetNeighbour(i);

            // if there isn't, we need not do anything for this direction
            if (!rna)
                continue;

            // get ID of adjacent node
            unsigned int xaid = rna->coord_id;

            // calculate new path length for this node
            unsigned int new_way_length = pf_nodes[best_id].way_length + best->routes[i]->GetLength();

            // is this step forbidden?
            // [TODO: check what "forbidden" means]
            // [TODO: this can probably moved up a bit (improve fail-fast behavior]
            if (best->routes[i] == forbidden)
                continue;

            // no detours over/through buildings, except for harbors and goals
            if ((i == 1) && (rna->GetGOT() != GOT_FLAG) && (rna != goal) &&
                    (rna->GetGOT() != GOT_NOB_HARBORBUILDING)) {
                continue;
            }

            // in ware mode, we have to add penalty points for overloaded carriers, so the
            // algorithm will select alternative routes
            if (ware_mode) {
                new_way_length += best->GetPunishmentPoints(i);
            } else if (best->routes[i]->GetRoadType() == RoadSegment::RT_BOAT) {
                // otherwise (if this is NOT in ware mode), ignore this node if it's a water road
                // (boat segment), because only wares can be transported there, humans cannot use
                // them
                continue;
            }

            // if the current route is longer than the maximum, ignore this segment
            if (new_way_length > max)
                continue;

            // if we have already checked this node before
            if (pf_nodes[xaid].lastVisited == currentVisit) {
                // then just update route length and predecessor, if the way is shorter
                if (new_way_length < pf_nodes[xaid].way_length) {
                    pf_nodes[xaid].way_length  = new_way_length;
                    pf_nodes[xaid].pred = best_id;
                    rna->estimate = rna->distance + new_way_length;
                    todo.rearrange(rna);
                    pf_nodes[xaid].dir = i;
                    pf_nodes[xaid].count_nodes = pf_nodes[best_id].count_nodes + 1;
                }
                continue;
            }

            // everything's fine, expand this node
            pf_nodes[xaid].lastVisited = currentVisit;
            pf_nodes[xaid].count_nodes = pf_nodes[best_id].count_nodes + 1;
            pf_nodes[xaid].way_length = new_way_length;
            pf_nodes[xaid].dir = i;
            pf_nodes[xaid].pred = best_id;

            rna->distance = CalcDistance(rna->GetX(), rna->GetY(), PathfindingPoint::dst_x,
                    PathfindingPoint::dst_y);
            rna->estimate = rna->distance + new_way_length;

            todo.push(rna);
        }

        // are we currently on a harbor?
        if (best->GetGOT() == GOT_NOB_HARBORBUILDING) {
            std::vector<nobHarborBuilding::ShipConnection> scs;
            static_cast<const nobHarborBuilding*>(best)->GetShipConnections(scs);

            for (unsigned i = 0; i < scs.size(); ++i) {
                // get ID of adjacent node
                unsigned xaid = scs[i].dest->coord_id;

                // calculate new path for this node
                unsigned new_way_length = pf_nodes[best_id].way_length  + scs[i].way_costs;

                // if the current route is longer than the maximum, ignore this segment
                if (new_way_length > max)
                    continue;

                // if we have already checked this node before
                if (pf_nodes[xaid].lastVisited == currentVisit) {
                    // then just update route length and predecessor, if the way is shorter
                    if (new_way_length < pf_nodes[xaid].way_length)
                    {
                        pf_nodes[xaid].way_length  = new_way_length;
                        pf_nodes[xaid].pred = best_id;
                        scs[i].dest->estimate = scs[i].dest->distance + new_way_length;
                        todo.rearrange(scs[i].dest);
                        pf_nodes[xaid].dir = 100;
                        pf_nodes[xaid].count_nodes = pf_nodes[best_id].count_nodes + 1;
                    }

                    continue;
                }

                // everything's fine, expand this node
                pf_nodes[xaid].lastVisited = currentVisit;
                pf_nodes[xaid].count_nodes = pf_nodes[best_id].count_nodes + 1;
                pf_nodes[xaid].way_length = new_way_length;
                pf_nodes[xaid].dir = 100;
                pf_nodes[xaid].pred = best_id;

                scs[i].dest->distance = CalcDistance(scs[i].dest->GetX(), scs[i].dest->GetY(),
                        PathfindingPoint::dst_x, PathfindingPoint::dst_y);
                scs[i].dest->estimate = scs[i].dest->distance + new_way_length;

                todo.push(scs[i].dest);
            }
        }
    }

    // done, but to-do list is empty (no more available node) and no path to destination was found
    //   --> no path exists
    if (record)
        GameClient::inst().AddPathfindingResult(0xff, length, next_harbor);
    return false;
}



/**
 * Checks if free route is still passable, and returns end node of route
 *
 * @param x_start
 * @param y_start
 * @param route
 * @param pos
 * @param IsNodeOK
 * @param IsNodeToDestOk
 * @param x_dest
 * @param y_dest
 * @param param
 * @return
 */
bool GameWorldBase::CheckFreeRoute(
        const MapCoord x_start,
        const MapCoord y_start,
        const std::vector<unsigned char>& route,
        const unsigned pos,
        FP_Node_OK_Callback IsNodeOK,
        FP_Node_OK_Callback IsNodeToDestOk,
        MapCoord* x_dest,
        MapCoord* y_dest,
        const void * const param) const
{
    MapCoord x = x_start, y = y_start;

    assert(pos < route.size());

    for (unsigned i = pos; i < route.size(); ++i) {
        GetPointA(x, y, route[i]);
        if (!IsNodeToDestOk(*this, x, y, route[i], param))
            return false;
        if (i < route.size()-1 && !IsNodeOK(*this, x, y, route[i], param))
            return false;
    }

    if (x_dest)
        *x_dest = x;
    if (y_dest)
        *y_dest = y;

    return true;
}

/** parameter struct for free pathfinding for road construction */
struct Param_RoadPath {
    /** are we building a water road? */
    bool boat_road;
};

/**
 * Continue condition for road construction pathfinding
 *
 * @param gwb
 * @param x
 * @param y
 * @param dir
 * @param param
 * @return
 */
bool IsPointOK_RoadPath(const GameWorldBase& gwb, const MapCoord x, const MapCoord y,
        const unsigned char dir, const void *param) {
    // cast parameter into right type, so we can access content
    const Param_RoadPath * prp = static_cast<const Param_RoadPath*>(param);

    // can we build a road on this node?
    if (!gwb.RoadAvailable(prp->boat_road, x, y, dir))
        return false;

    // is this node in current player's territory?
    if(!gwb.IsPlayerTerritory(x, y))
        return false;

    return true;
}

/**
 * road construction Pathfinding (free pathfinding with some special parameters)
 *
 * @param x_start
 * @param y_start
 * @param x_dest
 * @param y_dest
 * @param route
 * @param boat_road
 * @return
 */
bool GameWorldViewer::FindRoadPath(
        const MapCoord x_start,
        const MapCoord y_start,
        const MapCoord x_dest,
        const MapCoord y_dest,
        std::vector<unsigned char>& route,
        const bool boat_road)
{
    Param_RoadPath prp = { boat_road };
    return FindFreePath(x_start, y_start, x_dest, y_dest,
            false,              //no random route
            100,                //max route length = 100
            &route,             //route output
            NULL, NULL,         //don't care about route length and start direction
            IsPointOK_RoadPath, //function to check single node step
            NULL,               //no function to check step to goal node
            &prp,               //param for IsPointOK_RoadPath ('is this water road?')
            false);             //do not record this
}

/**
 * continue condition for free path for humans
 *
 * @param gwb
 * @param x
 * @param y
 * @param dir
 * @param param
 * @return
 */
bool IsPointOK_HumanPath(const GameWorldBase& gwb, const MapCoord x, const MapCoord y,
        const unsigned char dir, const void *param) {
    // is this node passable?
    noBase::BlockingManner bm = gwb.GetNO(x,y)->GetBM();
    if (bm != noBase::BM_NOTBLOCKING && bm != noBase::BM_TREE && bm != noBase::BM_FLAG)
        return false;

    return true;
}

/**
 * Additional continue-condition for free paths for humans, which must hold for the last vertex to
 * the goal node
 *
 * @param gwb
 * @param x
 * @param y
 * @param dir
 * @param param
 * @return
 */
bool IsPointToDestOK_HumanPath(const GameWorldBase& gwb, const MapCoord x, const MapCoord y,
        const unsigned char dir, const void *param) {
    // is this node passable?
    // don't walk over water, lava or marsh
    if (!gwb.IsNodeToNodeForFigure(x,y,(dir+3)%6))
        return false;

    return true;
}

/**
 * pathfinding for humans (free pathfinding with special parameters);
 * returns the direction for the next step
 *
 * @param x_start
 * @param y_start
 * @param x_dest
 * @param y_dest
 * @param max_route
 * @param random_route
 * @param length
 * @param record
 * @return
 */
unsigned char GameWorldBase::FindHumanPath(
        const MapCoord x_start,
        const MapCoord y_start,
        const MapCoord x_dest,
        const MapCoord y_dest,
        const unsigned max_route,
        const bool random_route,
        unsigned *length,
        const bool record) const
{
    // read from replay?
    if (GameClient::inst().ArePathfindingResultsAvailable() && !random_route) {
        unsigned char dir;
        if (GameClient::inst().ReadPathfindingResult(&dir, length, NULL))
            return dir;
    }

    unsigned char first_dir = 0xFF;
    FindFreePath(x_start,y_start,x_dest,y_dest,
            random_route,               //random start direction yes/no
            max_route,                  //max length of route
            NULL,                       //do not save actual route
            length,                     //get length of actual route
            &first_dir,                 //get the direction of the first step (return it below)
            IsPointOK_HumanPath,        //function to check single node step
            IsPointToDestOK_HumanPath,  //function to check step to goal node
            NULL,                       //no params for checking functions required
            record);                    //record this, yes/no

    if (!random_route) //[TODO: shouldn't this be checking "record"?]
        GameClient::inst().AddPathfindingResult(first_dir, length, NULL);

    return first_dir;
}

/**
 * continue condition for free path for ships
 *
 * @param gwb
 * @param x
 * @param y
 * @param dir
 * @param param
 * @return
 */
bool IsPointOK_ShipPath(const GameWorldBase& gwb, const MapCoord x, const MapCoord y,
        const unsigned char dir, const void *param) {
    // is the terrain around this node water?
    for (unsigned i = 0; i < 6; ++i) {
        if (gwb.GetTerrainAround(x, y, i) != TT_WATER)
            return false;
    }

    return true;
}

/**
 * Additional continue condition for free paths for ships, which must hold for the last vertext to
 * the goal node
 *
 * @param gwb
 * @param x
 * @param y
 * @param dir
 * @param param
 * @return
 */
bool IsPointToDestOK_ShipPath(const GameWorldBase& gwb, const MapCoord x, const MapCoord y,
        const unsigned char dir, const void *param) {
    // the terrain ahead and behind must be water
    if (gwb.GetWalkingTerrain1(x, y, (dir+3)%6) == TT_WATER &&
            gwb.GetWalkingTerrain2(x, y, (dir+3)%6) == TT_WATER)
        return true;
    else
        return false;
}

/**
 * pathfinding for ships on water (free pathfinding with special parameters)
 *
 * @param x_start
 * @param y_start
 * @param x_dest
 * @param y_dest
 * @param route
 * @param length
 * @param max_length
 * @param cb
 * @return
 */
bool GameWorldBase::FindShipPath(
        const MapCoord x_start,
        const MapCoord y_start,
        const MapCoord x_dest,
        const MapCoord y_dest,
        std::vector<unsigned char> * route,
        unsigned * length,
        const unsigned max_length,
        GameWorldBase::CrossBorders * cb)
{
    return FindFreePath(x_start, y_start, x_dest, y_dest,
            true,                       //start route with random direction
            400,                        //max route length is 400
            route,                      //save resulting route
            length,                     //save resulting route length
            NULL,                       //don't save direction of first step
            IsPointOK_ShipPath,         //function to check single node step
            IsPointToDestOK_ShipPath,   //function to check step to goal node
            NULL,                       //no params for checking functions required
            false);                     //do not record this
}

/**
 * Check if ship's route is still valid (uses `GameWorldBase::CheckFreeRoute()`)
 *
 * @param x_start
 * @param y_start
 * @param route
 * @param pos
 * @param x_dest
 * @param y_dest
 * @return
 */
bool GameWorldGame::CheckShipRoute(
        const MapCoord x_start,
        const MapCoord y_start,
        const std::vector<unsigned char>& route,
        const unsigned pos,
        MapCoord* x_dest,
        MapCoord* y_dest)
{
    return CheckFreeRoute(x_start,y_start,
            route,                      //the route to check
            pos,                        //current position in the route (node list)
            IsPointOK_ShipPath,         //function to check single node step
            IsPointToDestOK_ShipPath,   //function to check step to goal node
            x_dest,
            y_dest,
            NULL);                      //no params for checking functions required
}

/**
 * continue condition for trade paths
 *
 * @param gwb
 * @param x
 * @param y
 * @param dir
 * @param param
 * @return
 */
bool IsPointOK_TradePath(const GameWorldBase& gwb, const MapCoord x, const MapCoord y,
        const unsigned char dir, const void *param) {
    //node passable?
    noBase::BlockingManner bm = gwb.GetNO(x,y)->GetBM();
    if (bm != noBase::BM_NOTBLOCKING && bm != noBase::BM_TREE && bm != noBase::BM_FLAG)
        return false;

    //get owner of current node
    unsigned char player = gwb.GetNode(x,y).owner;
    //Ally or no player? Then ok
    if (player == 0 || gwb.GetPlayer(*((unsigned char*)param))->IsAlly(player-1))
        return true;
    else
        return false;
}

/**
 * Additional continue condition for trade paths, which must hold for the last vertext to the
 * goal node
 *
 * @param gwb
 * @param x
 * @param y
 * @param dir
 * @param param
 * @return
 */
bool IsPointToDestOK_TradePath(const GameWorldBase& gwb, const MapCoord x, const MapCoord y,
        const unsigned char dir, const void *param) {
    //is node passable?
    //don't walk over water, lava or marsh
    if (!gwb.IsNodeToNodeForFigure(x, y, (dir+3)%6))
        return false;

    //Make sure not to go through hostile territory
    //Get owners of previous node (old_player) and current node (new_player)
    unsigned char
        old_player = gwb.GetNode(gwb.GetXA(x, y, (dir+3)%6), gwb.GetYA(x, y, (dir+3)%6)).owner,
        new_player = gwb.GetNode(x,y).owner;
    // Ally or no player? Then ok
    if (new_player == 0 || gwb.GetPlayer(*((unsigned char*)param))->IsAlly(new_player-1)) {
        return true;
    } else {
        // is previous node still un-owned or owned by ally?
        if (old_player != 0 && !gwb.GetPlayer(*((unsigned char*)param))->IsAlly(old_player-1))
            return true;
        else //both current and previous nodes are owned by enemies, we can't go there
            return false;
    }
}


/**
 * Find a route for trade caravans (free pathfinding with some preparation and special parameters)
 *
 * @param start
 * @param dest
 * @param player
 * @param max_route
 * @param random_route
 * @param route
 * @param length
 * @param record
 * @return
 */
unsigned char GameWorldGame::FindTradePath(
    const Point<MapCoord> start,
    const Point<MapCoord> dest,
    const unsigned char player,
    const unsigned max_route,
    const bool random_route,
    std::vector<unsigned char> * route,
    unsigned *length,
    const bool record) const
{
    //Get owner of destination node
    unsigned char pp = GetNode(dest.x,dest.y).owner;
    //If node is owned by an enemy (i.e. not by nobody and not by an ally)
    if (!(pp == 0 || GetPlayer(player)->IsAlly(pp-1)))
        return 0xff; //then we can't get there

    //check if destination is a warehouse
    bool is_warehouse_at_goal = false;
    if (GetNO(dest.x,dest.y)->GetType() == NOP_BUILDING) {
        if (GetSpecObj<noBuilding>(dest.x,dest.y)->IsWarehouse())
            is_warehouse_at_goal = true;
    }

    //if humans cannot go to destination (e.g. off the map or object blocking that node) and there
    //is no warehouse there (which may block the location otherwise)
    if (!IsNodeForFigures(dest.x,dest.y) && !is_warehouse_at_goal)
        return 0xff; //then we can't get there

    //look for a route and get the first direction out of it
    unsigned char first_dir = 0xFF;
    FindFreePath(start.x, start.y, dest.x, dest.y,
            random_route,
            max_route,
            route,
            length,
            &first_dir,
            IsPointOK_TradePath,
            IsPointToDestOK_TradePath,
            &player,
            record);

    //return the first direction
    return first_dir;
}

/**
 * Check whether trade path is still valid
 *
 * @param start
 * @param route
 * @param pos
 * @param player
 * @param dest
 * @return
 */
bool GameWorldGame::CheckTradeRoute(
        const Point<MapCoord> start,
        const std::vector<unsigned char>& route,
        const unsigned pos,
        const unsigned char player,
        Point<MapCoord> * dest) const
{
    return CheckFreeRoute(start.x, start.y,
            route,
            pos,
            IsPointOK_TradePath,
            IsPointToDestOK_HumanPath,
            dest ? &dest->x : NULL,
            dest ? &dest->y : NULL,
            &player);
}

/**
 * pathfinding for humans on roads (pathfinding on roads with special parameters);
 * returns the direction for the next step
 *
 * @param start
 * @param goal
 * @param length
 * @param next_harbor
 * @param forbidden
 * @return
 */
unsigned char GameWorldGame::FindHumanPathOnRoads(
        const noRoadNode * const start,
        const noRoadNode * const goal,
        unsigned * length,
        Point<MapCoord> * next_harbor,
        const RoadSegment * const forbidden)
{
    unsigned char first_dir = 0xFF;
    if (FindPathOnRoads(start, goal, /*ware-mode:*/false, length, &first_dir, next_harbor,
            forbidden))
        return first_dir;
    else
        return 0xFF;
}

/**
 * pathfinding for wares on roads (pathfinding on roads with special parameters);
 * returns the direction for the next step
 *
 * @param start
 * @param goal
 * @param length
 * @param next_harbor
 * @param max
 * @return
 */
unsigned char GameWorldGame::FindPathForWareOnRoads(
        const noRoadNode * const start,
        const noRoadNode * const goal,
        unsigned * length,
        Point<MapCoord> * next_harbor,
        unsigned max)
{
    unsigned char first_dir = 0xFF;
    if (FindPathOnRoads(start, goal, /*ware-mode:*/true, length, &first_dir, next_harbor,
            /*no forbidden parts*/NULL, /*record:*/true, max))
        //[TODO: check if the hard-coded TRUE for "record" in the call above is right]
        return first_dir;
    else
        return 0xFF;
}
