// application.cpp <Starter Code>
// <Your name>
//
//
// Adam T Koehler, PhD
// University of Illinois Chicago
// CS 251, Fall 2023
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//
// 
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <iostream>
#include <iomanip>  /*setprecision*/
#include <string>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstring>
#include <cassert>

#include "tinyxml2.h"
#include "dist.h"
#include "graph.h"
#include "osm.h"
#include <limits>
#include <bits/stdc++.h>

using namespace std;
using namespace tinyxml2;

//
// Implement your standard application here
//

class prioritize  // you could also use a struct
{
public:
  bool operator()(const pair<long long,double>& p1, const pair<long long,double>& p2) const
  {
    return p1.second > p2.second; 
  }
};


//global Variable
const double INF = numeric_limits<double>::max();

BuildingInfo searchBuilding(vector<BuildingInfo> buildings, string query){
  //
  //2x linear search for the abbreviation then if contains as substring;
  //
  unsigned int i;
  for (i =0; i < buildings.size(); i++){
    if (buildings[i].Abbrev == query)
      return buildings[i];
  }

  for (i = 0; i < buildings.size(); i++){
    if (buildings[i].Fullname.find(query) != string::npos)
      return buildings[i];
  }

  BuildingInfo empty;
  return empty;
}

void PrintBuilding(BuildingInfo building, string person){
  cout << person << endl;
  string what = building.Abbrev;

  if (building.Fullname != ""){

    cout << " " << building.Fullname << endl;
    Coordinates  Loc = building.Coords;
    cout << " (" << Loc.Lat << ", " << Loc.Lon << ")" << endl;
  }
  //whats the case I dont find anything?
}

Coordinates finalDestination(BuildingInfo one, BuildingInfo two){
  Coordinates Cord1 = one.Coords;
  Coordinates Cord2 = two.Coords;
  return centerBetween2Points(Cord1.Lat, Cord1.Lon, Cord2.Lat, Cord2.Lon);
}

double FindMinDistance(Coordinates C, vector<BuildingInfo> Buildings, BuildingInfo& midBuilding, double Otherbuilding){
  double min = INFINITY;
  for (int i =0; i < Buildings.size(); i++){
    Coordinates OtherCord = Buildings[i].Coords;
    double distance = distBetween2Points(C.Lat, C.Lon, OtherCord.Lat, OtherCord.Lon);
    if (distance < min && distance > Otherbuilding){
      midBuilding = Buildings[i];
      min = distance;
    }
  }
  return min;
}

Coordinates nearestNode(  BuildingInfo building, 
                          vector<FootwayInfo> Footways,
                          map<long long, Coordinates> Nodes
          ){

  Coordinates nearestNode;
  double min = INF;
  Coordinates C = building.Coords;
  for (int i =0; i < Footways.size(); i++){
    // if (Footways[i].ID == building.)
    for (int k = 0; k < Footways[i].Nodes.size(); k++){
      Coordinates C2 = Nodes.at(Footways[i].Nodes[k]);
      double distance = distBetween2Points( C2.Lat, C2.Lon, C.Lat, C.Lon);
      if (distance < min ){
        nearestNode = C2;
        min = distance;
      }
    }  
  }
  
  return nearestNode;
}


bool PrintPath(Coordinates C3, Coordinates endLocation, map<long long, long long> Pred, map<long long, double> dist, string person){
  long long CurrLocation = endLocation.ID;

  if (dist[endLocation.ID] >= INF){
    return false;
  }
  else if(Pred.find(CurrLocation) == Pred.end()){
    return false;
  }
  else if(CurrLocation == C3.ID){
    cout <<  person << " distance to dest: 0 miles";
    cout << "Path: " << C3.ID;
  }else{
    cout << person << " distance to dest: " << dist[CurrLocation] << " miles" << endl;
    cout << "Path: " << CurrLocation;

    while (CurrLocation != C3.ID){
      if (CurrLocation != C3.ID){
        cout << "->";
      }
      cout << Pred[CurrLocation];
      CurrLocation = Pred[CurrLocation];
    }
  }
  cout << endl;
  return true;
}


map<long long,  long long> DijkstraShortestPath(graph<long long, double>& G, long long start, map<long long, double>& distances){
  //we have an initial start position; set its initial position to 0;
  map<long long,  long long> pred;

  vector<long long> unVisitedNodesVec = G.getVertices();

  priority_queue<
  pair<long long,double>,          // (key,value) pair
  vector<pair<long long,double>>,  // store pairs in a vector
  prioritize> unvisitedQueue;   // function object

  set<long long> Visited;
  set<long long> unVisited;
  for (int k =0; k < unVisitedNodesVec.size(); k++){
    distances[unVisitedNodesVec[k]] = INF;
    unvisitedQueue.push(make_pair(unVisitedNodesVec[k], INF));
  }

  
  //initialize the values of the start one;
  distances[start] = 0;
  unvisitedQueue.push(make_pair(start, 0));

  

  while (!unvisitedQueue.empty()){
    //get the first front of the list;
    pair<long long, double> topVertice = unvisitedQueue.top();
    unvisitedQueue.pop();

    if (distances[topVertice.first] == INF){
      continue;
    }
    else if(Visited.count(topVertice.first)){
      continue;
    }
    else{
        //if it;s not in the visited then we look for all neighbors
        // we then iterate through them if the dist lower then add to predessor list;
        //we also push back the neighbors into our worklist;
      Visited.insert(topVertice.first);
      set<long long> N = G.neighbors(topVertice.first);
      for (auto E: N){
        double edgeWeight;
        G.getWeight(topVertice.first, E, edgeWeight);
        double alternativeDistance = distances[topVertice.first] + edgeWeight;

        if (alternativeDistance < distances[E]){
          distances[E] = alternativeDistance;
          pred[E] = topVertice.first;
          unvisitedQueue.push(make_pair(E, alternativeDistance));
        }
      }
    }
  }
  //case that we start to start is just 0 distance;
  pred[start] = 0;
  return pred;
}

void PrintNearestNode(Coordinates Node, long long ID, string s){
  cout << s << endl;
  cout <<" " << ID << endl;
  cout << " " << "(" << Node.Lat << ", ";
  cout << Node.Lon << ")" << endl;
}


//this is the implementation that handles the print statements using other functions
//to do all the heavy lifting e.g. DijkstraShortestPath;
void application(
    map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways,
    vector<BuildingInfo>& Buildings, graph<long long, double>& G) {
  string person1Building, person2Building;

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);
  
  BuildingInfo buildingOne  = searchBuilding(Buildings, person1Building);

  while (person1Building != "#"){
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    
    person2Building.clear();
    getline(cin, person2Building);

    BuildingInfo buildingtwo  = searchBuilding(Buildings, person2Building);

    if (buildingtwo.Fullname == ""){
      cout << "Person 2's building not found" << endl;
      
    }
    else if (buildingOne.Fullname == ""){
      cout << "Person 1's building not found"<<endl;
    }
    else {
    //
    //mileStone 8;
    //Finding the nearest building;
    //
    Coordinates midCoordinate = finalDestination(buildingOne, buildingtwo);
    BuildingInfo buildingCenter;
    double min = FindMinDistance(midCoordinate, Buildings, buildingCenter, -1);

    PrintBuilding(buildingOne, "Person 1's point:");
    PrintBuilding(buildingtwo, "Person 2's point:");
    PrintBuilding(buildingCenter, "Destination Building:");


    cout << endl;

    Coordinates C1 =  nearestNode(buildingOne, Footways, Nodes);
    PrintNearestNode(C1, C1.ID, "Nearest P1 node:");
    
    Coordinates C2 = nearestNode(buildingtwo, Footways, Nodes);
    PrintNearestNode(C2, C2.ID, "Nearest P2 node:");

    Coordinates C3 = nearestNode(buildingCenter, Footways, Nodes);
    PrintNearestNode(C3, C3.ID, "Nearest destination node:");
    
    // TO DO: lookup buildings, find nearest start and dest nodes, find center
    // run Dijkstra's alg from each start, output distances and paths to destination:
    //
    //
    map<long long, double> dist1;
    map<long long, long long> Pred1 = DijkstraShortestPath(G,C3.ID, dist1);

    if (dist1[C1.ID] >= INF || dist1[C2.ID] >= INF){
      cout << endl <<  "Sorry, destination unreachable." << endl << endl;
    }
    else {
      cout << endl;
      PrintPath(C3,C1,Pred1, dist1, "Person 1's");

      cout << endl;
      PrintPath(C3, C2, Pred1, dist1,  "Person 2's");
    }
  }
    // another navigation?
    //
    
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    person1Building.clear();
    getline(cin, person1Building);
    buildingOne = searchBuilding(Buildings, person1Building);
  }    
}

int main() {
  graph<long long, double> G;

  // maps a Node ID to it's coordinates (lat, lon)
  map<long long, Coordinates>  Nodes;
  // info about each footway, in no particular order
  vector<FootwayInfo>          Footways;
  // info about each building, in no particular order
  vector<BuildingInfo>         Buildings;
  XMLDocument                  xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  // }
  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;


  //
  // TO DO: build the graph, output stats:
  //

  for (auto elememt: Nodes){
    G.addVertex(elememt.first);
  }

  //
  // adding the edges;
  // this would be the seonc to last;
  //
  for (int i =0; i < Footways.size(); i++){
    //how do we get the distance?
    for (int k = 0; k < Footways[i].Nodes.size() -1; k++){
      //calc distance given key from Foot->Nodes[k]
      //Issue we calculate based on distance it should be based on ID???
      Coordinates C1 = Nodes.at(Footways[i].Nodes[k]);
      Coordinates C2 = Nodes.at(Footways[i].Nodes[k+1]);
      double distance = distBetween2Points(C1.Lat, C1.Lon, C2.Lat, C2.Lon);
      //then get distance for both sides;
      G.addEdge(Footways[i].Nodes[k], Footways[i].Nodes[k+1], distance);
      G.addEdge(Footways[i].Nodes[k+1], Footways[i].Nodes[k], distance);
    }
    //I need the foot ways info such as I need ...long long;
  }

  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  

  // Execute Application
  application(Nodes, Footways, Buildings, G);

  //
  // done:
  //
  cout << "** Done **" << endl;
  return 0;
}


