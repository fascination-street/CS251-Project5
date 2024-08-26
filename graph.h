#pragma once

#include <iostream>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>
#include <utility>

using namespace std;

/// @brief Simple directed graph using an adjacency list.
/// @tparam VertexT vertex type
/// @tparam WeightT edge weight type
template <typename VertexT, typename WeightT>
class graph {
   private:
   //typedef pair<VertexT, WeightT> vertices;
   map<VertexT, map<VertexT, WeightT>> adj;

   //size_t numEdges;
   size_t numVertices;

   public:
    /// Default constructor
    graph() {
        //numEdges = 0;
        numVertices = 0;
    }

    /// @brief Add the vertex `v` to the graph, must run in at most O(log |V|).
    /// @param v
    /// @return true if successfully added; false if it existed already
    bool addVertex(VertexT v) {
        map<VertexT, WeightT> empt; //construct empty map for new vertex
        auto tf = adj.emplace(v, empt); //add vertex and empty map to the adjacency list
        ++numVertices;
        return tf.second;
    }  

    /// @brief Add or overwrite directed edge in the graph, must run in at most O(log |V|).
    /// @param from starting vertex
    /// @param to ending vertex
    /// @param weight edge weight / label
    /// @return true if successfully added or overwritten;
    ///         false if either vertices isn't in graph
    bool addEdge(VertexT from, VertexT to, WeightT weight) {
        //return false if either vertex is not in the list
        if (adj.find(from) == adj.end() || adj.find(to) == adj.end()) {return false;}

        map<VertexT, WeightT>* s = &(adj.at(from)); //get map address in memory so we can alter it
        s->erase(to); //remove to from the map so we can overwrite
        auto tf = s->emplace(to, weight); //add the new edge to the map

        //++numEdges; //you think you know me

        return tf.second;
    }

    /// @brief Maybe get the weight associated with a given edge, must run in at most O(log |V|).
    /// @param from starting vertex
    /// @param to ending vertex
    /// @param weight output parameter
    /// @return true if the edge exists, and `weight` is set;
    ///         false if the edge does not exist
    bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
        if (adj.find(from) == adj.end()) {return false;} //if from isnt in the list, return false
        const map<VertexT, WeightT>* m = &(adj.at(from)); //save from's map to prevent extra function calls, use a pointer to not have to copy the map ;)
        if (m->find(to) == m->end()) {return false;} //if there is no edge then return false

        weight = m->at(to); 
        return true;
    }

    /// @brief Get the out-neighbors of `v`. Must run in at most O(|V|).
    /// @param v
    /// @return vertices that v has an edge to
    set<VertexT> neighbors(VertexT v) const {
        set<VertexT> S;
        map<VertexT, WeightT> m = adj.at(v);
        for (const auto& it : m) {
            S.emplace(it.first);
        }
        return S;
    }

    /// @brief Return a vector containing all vertices in the graph
    vector<VertexT> getVertices() const {
        vector<VertexT> vert;
        for (const auto& it : adj) {
            vert.push_back(it.first);
        }
        return vert;
    }

    /// @brief Get the number of vertices in the graph. Runs in O(1).
    size_t NumVertices() const {
        return numVertices;
    }

    /// @brief Get the number of directed edges in the graph. Runs in at most O(|V|).
    size_t NumEdges() const {
        size_t numEdges = 0;

        for (const auto& a : adj) {numEdges += a.second.size();}

        return numEdges;
    }
};
