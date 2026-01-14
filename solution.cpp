#include <algorithm>
#include <cassert>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

template <class T>
struct Compressor {
  private:
    std::vector<T> data;

  public:
    Compressor() = default;

    void add(const T &x) {
        data.emplace_back(x);
    }

    void build() {
        std::sort(data.begin(), data.end());
        data.erase(std::unique(data.begin(), data.end()), data.end());
    }

    size_t size() const { return data.size(); }

    int operator()(const T &x) const {
        int id = std::lower_bound(data.begin(), data.end(), x) - data.begin();
        assert(data.at(id) == x);
        return id;
    }

    const T &operator[](int k) const {
        assert(0 <= k && k < size());
        return data[k];
    }
};

template <class T>
struct Edge {
    int start, end;
    T distance;

    Edge() = default;
    Edge(int _start, int _end, T _distance) :
        start(_start), end(_end), distance(_distance) {}
};

template <class T>
std::vector<Edge<T>> ReadEdges() {
    std::vector<Edge<T>> edges;
    std::string line;
    
    while (std::getline(std::cin, line)) {
        std::stringstream ss(line);

        int start, end;
        long double distance;
        char comma;

        ss >> start >> comma >> end >> comma >> distance;

        Edge<T> edge(start, end, distance);
        edges.emplace_back(edge);

        Edge<T> rev_edge(end, start, distance);
        edges.emplace_back(rev_edge);
    }
    
    return edges;
}

template <class T>
Compressor<int> CompressId(const std::vector<Edge<T>> &edges) {
    Compressor<int> compressor;
    for (const Edge<T> &edge : edges) {
        compressor.add(edge.start);
        compressor.add(edge.end);
    }
    compressor.build();
    return compressor;
}

template <class T>
using Graph = std::vector<std::vector<Edge<T>>>;

template <class T>
Graph<T> GetGraph(const std::vector<Edge<T>> &edges, const Compressor<int> &compressor) {
    int node_num = compressor.size();
    
    Graph<T> graph(node_num);
    
    for (const Edge<T> &edge : edges) {
        int start = compressor(edge.start);
        int end = compressor(edge.end);
        T distance = edge.distance;
        
        Edge<T> new_edge(start, end, distance);
        graph.at(start).emplace_back(new_edge);
    }
    
    return graph;
}

template <class T>
bool ChangeMax(T &current_value, const T &new_value) {
    if (current_value < new_value) {
        current_value = new_value;
        return true;
    } else {
        return false;
    }
}

template <class T>
std::vector<int> BitDP(const Graph<T> &graph) {
    int node_num = graph.size();
    std::vector<std::vector<T>> dp_table(1 << node_num, std::vector<T>(node_num, -1));
    std::vector<std::vector<int>> previous_transition(1 << node_num, std::vector<int>(node_num, -1));
    
    for (int node = 0; node < node_num; node++) {
        dp_table[1 << node][node] = 0;
    }
    
    for (int set = 0; set < (1 << node_num); set++) {
        for (int node = 0; node < node_num; node++) {
            if (dp_table[set][node] == -1) { 
                continue; 
            }
            for (const Edge<T> &edge : graph.at(node)) {
                int next_node = edge.end;
                T distance = edge.distance;
                if (set & (1 << next_node)) {
                    continue;
                }
                int next_set = set | (1 << next_node);
                if (ChangeMax(dp_table[next_set][next_node], dp_table[set][node] + distance)) {
                    previous_transition[next_set][next_node] = node;
                }
            }
        }
    }

    T longest_distance = 0;
    int current_set, current_node;
    for (int set = 0; set < (1 << node_num); set++) {
        for (int node = 0; node < node_num; node++) {
            if (ChangeMax(longest_distance, dp_table[set][node])) {
                current_set = set;
                current_node = node;
            }
        }
    }

    std::vector<int> route;
    while (current_node != -1) {
        route.emplace_back(current_node);
        int previous_node = previous_transition[current_set][current_node];
        current_set ^= (1 << current_node);
        current_node = previous_node;
    }

    return route;
}

template <class T>
std::vector<int> solve_small(const std::vector<Edge<T>> &edges, 
                                const Compressor<int> &id_compressor) {
    Graph<T> graph = GetGraph(edges, id_compressor);
    
    std::vector<int> route = BitDP(graph);

    for (int &node : route) {
        node = id_compressor[node];
    }

    return route;
}

int main() {
    std::vector<Edge<long double>> edges = ReadEdges<long double>();
    
    Compressor<int> id_compressor = CompressId(edges);

    int node_num = id_compressor.size();

    std::vector<int> route;

    if (node_num <= 16) {
        route = solve_small(edges, id_compressor);
    }

    for (int node : route) {
        std::cout << node << "\r\n";
    }

    return 0;
}
