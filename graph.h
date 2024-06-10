// graph.h <Starter Code>
// < Your name >
//
// Basic graph class using adjacency matrix representation.  Currently
// limited to a graph with at most 100 vertices.
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

//We create an adjacency List my preference is (map<char>, sets<char>);

#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <set>
#include<string>
#include <map>


using namespace std;

template<typename VertexT, typename WeightT>
class graph {
 private:   
    map<VertexT, map<VertexT, WeightT>> NewAdjList;
    int edges;
 public:

  graph& operator=(const graph& other){
    if (this != &other){
      NewAdjList.clear();

      for (const auto& element: other.NewAdjList){
        VertexT vertex = element.first;
        map<VertexT, WeightT> CopyNeighbor = element.second;
        NewAdjList[vertex] = CopyNeighbor;
      }
    }
    return *this;
  }

  //
  // constructor:
  //
  // Constructs an empty graph where n is the max # of vertices
  // you expect the graph to contain.
  //
  // NOTE: the graph is implemented using an adjacency matrix.
  // If n exceeds the dimensions of this matrix, an exception
  // will be thrown to let you know that this implementation
  // will not suffice.
  //
  // -graph(int n) {
  //   if (n > MatrixSize) {
  //     throw runtime_error("graph::constructor: n exceeds internal matrix size");
  //   }
  // -}

  //constructor
  graph(){
    //I dont think we do anything;
  }
  //
  // NumVertices
  //
  // Returns the # of vertices currently in the graph.
  //
  int NumVertices() const {
    return NewAdjList.size();
  }

  //
  // NumEdges
  //
  // Returns the # of edges currently in the graph.
  //
  int NumEdges() const {
    int count = 0;
    //
    // loop through the adjacency matrix and count how many
    // edges currently exist:
    //
    for (auto& M: NewAdjList){
      auto vertex = M.first;
      for(auto& values: M.second){
          count++;
      }
    }
    return count;
  }

  //
  // addVertex
  //
  // Adds the vertex v to the graph if there's room, and if so
  // returns true.  If the graph is full, or the vertex already
  // exists in the graph, then false is returned.
  //
  bool addVertex(VertexT v) {
    if (NewAdjList.find(v) != NewAdjList.end()){
      return false;
    }
    NewAdjList[v];
    return true;
  }
  
  //adds the weight;
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    auto one = NewAdjList.find(from);
    auto two = NewAdjList.find(to);
    if (one != NewAdjList.end() && two != NewAdjList.end()){
      NewAdjList[from][to] = weight;
      return true;
    }

    //NewAdjList[to][from] = weight;
    return false;
  }

  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    //checking if the keys valid;
    auto V = NewAdjList.find(from);
    if (V != NewAdjList.end()){
      auto edge = V->second.find(to);
       if (edge != V->second.end()){
          weight = edge->second;
          return true;
       }
    }
    return false;
  }

  //
  // neighbors
  //
  // Returns a set containing the neighbors of v, i.e. all
  // vertices that can be reached from v along one edge.
  // Since a set is returned, the neighbors are returned in
  // sorted order; use foreach to iterate through the set.
  //
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT> S;
    auto key = NewAdjList.find(v);
    if (key != NewAdjList.end()){

      for (const auto& pair : key->second){
        S.insert(pair.first);
      }
    }
    return S;
  }

  //
  // getVertices
  //
  // Returns a vector containing all the vertices currently in
  // the graph.
  //
  vector<VertexT> getVertices() const {
      vector<VertexT> V;
      for (auto e: NewAdjList){
        V.push_back(e.first);
      }
      return V;
  }

  //
  // dump
  //
  // Dumps the internal state of the graph for debugging purposes.
  //
  // Example:
  //    graph<string,int>  G(26);
  //    ...
  //    G.dump(cout);  // dump to console
  //
  void dump(ostream& output) const {
    output << "***************************************************" << endl;
    output << "********************* GRAPH ***********************" << endl;

    output << "**Num vertices: " << this->NumVertices() << endl;
    output << "**Num edges: " << this->NumEdges() << endl;

    output << endl;
    output << "**Vertices:" << endl;
    
    int i = 0;
    for (auto& e : NewAdjList){
      output << " " << i << ". " << e.first << endl;
      i++;
    }

    output << endl;
    output << "**Edges:" << endl;

    i = 0;
    for (auto element: NewAdjList){
      i++;
      output << " row " << i << ": ";
      const set<VertexT>& edges = neighbors(element.first);
      for (auto check: NewAdjList){
        int weight = 0;
        if (edges.find(check.first) == edges.end()){
          output << "F ";
        }
        else{
          getWeight(element.first, check.first, weight);
          output << "(T," << weight << ")";
        }
      }
      output << endl;
    }
    
    output << "**************************************************" << endl;
  }
};



///////////////
//priority_queue<int, greater<int>> pq;
//                    this compares and gives the least when popping the pq;
//                    pairs comapre by default;
//                    