#include "application.h"

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iomanip> /*setprecision*/
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dist.h"
#include "graph.h"
#include "osm.h"
#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;

double INF = numeric_limits<double>::max();

graph<long long, double> buildGraph(
    const map<long long, Coordinates>& Nodes,
    const vector<FootwayInfo>& Footways,
    const vector<BuildingInfo>& Buildings) {
    graph<long long, double> G;

    //add vertices to the graph
    for (const auto& n : Nodes) {G.addVertex(n.first);}
    for (const auto& b : Buildings) {G.addVertex(b.Coords.ID);}

    //loop through the footways to find adjacent nodes, then add edges for them
    for (const auto& f : Footways) {
        const vector<long long>& fn = f.Nodes;

        for (unsigned long long i = 0; i < fn.size() - 1; i++) {
            long long n1 = fn.at(i);
            long long n2 = fn.at(i+1);

            if (Nodes.find(n1) != Nodes.end() && Nodes.find(n2) != Nodes.end()) {
                Coordinates c1 = Nodes.at(n1);
                Coordinates c2 = Nodes.at(n2);

                double dist = distBetween2Points(c1.Lat, c1.Lon, c2.Lat, c2.Lon);

                G.addEdge(n1, n2, dist);
                G.addEdge(n2, n1, dist);
            }
        }
    }

    //loop through buildings to add edges from them to their closest point 
    for (const auto& b : Buildings) {
        long long bid = b.Coords.ID;

        for (const auto& n : Nodes) {
            if (n.second.OnFootway) {
                double dist = distBetween2Points(b.Coords.Lat, b.Coords.Lon, n.second.Lat, n.second.Lon);
                if (dist <= 0.041) {
                    G.addEdge(bid, n.first, dist);
                    G.addEdge(n.first, bid, dist);
                }
            }
        }
    }
    
    return G;
}

vector<long long> dijkstra(
    const graph<long long, double>& G,
    long long start,
    long long target,
    const set<long long>& ignoreNodes) {
    
    //for the prqueue
    class prioritize {
        public:
            bool operator()(const pair<long long, double>& p1,
                            const pair<long long, double>& p2) const {
                return p1.second > p2.second;
            }
    };

    priority_queue<pair<long long, double>, 
                    vector<pair<long long, double>>, 
                    prioritize> unvisited; //holds our closest nodes 
    map<long long, double> dists; //stores distances with nodes for easy access
    map<long long, long long> pred; //stores predecessor nodes in order to build the vector easier
    vector<long long> path;
    
    //building the two maps, populating dists with INF and predecessors with -1 as default values
    const vector<long long>& verts = G.getVertices();
    for (const auto& vert : verts) {
        if (vert == start) dists.emplace(vert, 0); //start should have a distance of 0
        else dists.emplace(vert, INF);
        pred.emplace(vert, -1); 
    }
    
    //put the start node on the priority queue to begin Dijkstra's Algorithm
    unvisited.push(make_pair(start, 0));

    while (!unvisited.empty()) {
        //get everything required from the shortest node, then pop it off the queue
        auto p = unvisited.top();
        long long pNode = p.first; //p(air)Node - the node with shortest distance to the last node
        double pDist = p.second; //p(air)Dist - "INF" if untouched or other value if altered by Dijkstra's previously
        unvisited.pop();

        //if we can't find the shortest node in the map, just continue with the next iteration
        if (dists.find(pNode) == dists.end()) continue;
        double currdist = dists.at(pNode); //currdist 
        
        if (pDist > currdist) continue; //node is too far away, just move on
        if (pNode == target) break; //we have reached the end, no need to continue from here

        //iterate through all of pNode's neighbors to find the one closest to it
        const set<long long>& nbrs = G.neighbors(pNode);
        for (const auto& nbr : nbrs) {
            //skip this iteration if we're at a building
            if (ignoreNodes.find(nbr) != ignoreNodes.end() && nbr != start && nbr != target) continue;

            //otherwise caclulate the distance between this node and its neighbor
            double newDist = 0.0;
            G.getWeight(pNode, nbr, newDist);
            newDist += currdist;

            //if the distance in the map is greater than our calc'd distance, we want to update the maps
            if (newDist < dists.at(nbr)) {
                dists.erase(nbr);
                dists.emplace(nbr, newDist);

                pred.erase(nbr);
                pred.emplace(nbr, pNode);

                //then push it onto the prqueue
                unvisited.push(make_pair(nbr, newDist));
            }
        }
    }  

    if (dists.at(target) == INF) return path; //we did NOT find a path, return empty

    //build the path vector. since we're adding it from the back we need to reverse it
    long long pathNode = target;
    while(pathNode != -1) {
        path.push_back(pathNode);
        pathNode = pred.at(pathNode);
    }
    reverse(path.begin(), path.end());

    return path;
}

double pathLength(const graph<long long, double>& G, const vector<long long>& path) {
    double length = 0.0;
    double weight;
    for (size_t i = 0; i + 1 < path.size(); i++) {
        bool res = G.getWeight(path.at(i), path.at(i + 1), weight);
        assert(res);
        length += weight;
    }
    return length;
}

void outputPath(const vector<long long>& path) {
    for (size_t i = 0; i < path.size(); i++) {
        cout << path.at(i);
        if (i != path.size() - 1) {
            cout << "->";
        }
    }
    cout << endl;
}

void application(
    const vector<BuildingInfo>& Buildings,
    const graph<long long, double>& G) {
    string person1Building, person2Building;

    set<long long> buildingNodes;
    for (const auto& building : Buildings) {
        buildingNodes.insert(building.Coords.ID);
    }

    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);

    while (person1Building != "#") {
        cout << "Enter person 2's building (partial name or abbreviation)> ";
        getline(cin, person2Building);

        //
        // find the building coordinates
        //
        bool foundP1 = false;
        bool foundP2 = false;
        Coordinates P1Coords, P2Coords;
        string P1Name, P2Name;

        for (const BuildingInfo& building : Buildings) {
            if (building.Abbrev == person1Building) {
                foundP1 = true;
                P1Name = building.Fullname;
                P1Coords = building.Coords;
            }
            if (building.Abbrev == person2Building) {
                foundP2 = true;
                P2Name = building.Fullname;
                P2Coords = building.Coords;
            }
        }

        for (const BuildingInfo& building : Buildings) {
            if (!foundP1 &&
                building.Fullname.find(person1Building) != string::npos) {
                foundP1 = true;
                P1Name = building.Fullname;
                P1Coords = building.Coords;
            }
            if (!foundP2 && building.Fullname.find(person2Building) != string::npos) {
                foundP2 = true;
                P2Name = building.Fullname;
                P2Coords = building.Coords;
            }
        }

        if (!foundP1) {
            cout << "Person 1's building not found" << endl;
        } else if (!foundP2) {
            cout << "Person 2's building not found" << endl;
        } else {
            cout << endl;
            cout << "Person 1's point:" << endl;
            cout << " " << P1Name << endl;
            cout << " (" << P1Coords.Lat << ", " << P1Coords.Lon << ")" << endl;
            cout << "Person 2's point:" << endl;
            cout << " " << P2Name << endl;
            cout << " (" << P2Coords.Lat << ", " << P2Coords.Lon << ")" << endl;

            string destName;
            Coordinates destCoords;

            Coordinates centerCoords = centerBetween2Points(
                P1Coords.Lat, P1Coords.Lon, P2Coords.Lat, P2Coords.Lon);

            double minDestDist = numeric_limits<double>::max();

            for (const BuildingInfo& building : Buildings) {
                double dist = distBetween2Points(
                    centerCoords.Lat, centerCoords.Lon,
                    building.Coords.Lat, building.Coords.Lon);
                if (dist < minDestDist) {
                    minDestDist = dist;
                    destCoords = building.Coords;
                    destName = building.Fullname;
                }
            }

            cout << "Destination Building:" << endl;
            cout << " " << destName << endl;
            cout << " (" << destCoords.Lat << ", " << destCoords.Lon << ")" << endl;

            vector<long long> P1Path = dijkstra(G, P1Coords.ID, destCoords.ID, buildingNodes);
            vector<long long> P2Path = dijkstra(G, P2Coords.ID, destCoords.ID, buildingNodes);

            // This should NEVER happen with how the graph is built
            if (P1Path.empty() || P2Path.empty()) {
                cout << endl;
                cout << "At least one person was unable to reach the destination building. Is an edge missing?" << endl;
                cout << endl;
            } else {
                cout << endl;
                cout << "Person 1's distance to dest: " << pathLength(G, P1Path);
                cout << " miles" << endl;
                cout << "Path: ";
                outputPath(P1Path);
                cout << endl;
                cout << "Person 2's distance to dest: " << pathLength(G, P2Path);
                cout << " miles" << endl;
                cout << "Path: ";
                outputPath(P2Path);
            }
        }

        //
        // another navigation?
        //
        cout << endl;
        cout << "Enter person 1's building (partial name or abbreviation), or #> ";
        getline(cin, person1Building);
    }
}
