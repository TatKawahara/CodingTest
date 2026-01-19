#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

template <class T>
class Edge {
  public:
    int start, end;
    T distance;

    Edge() = default;
    Edge(int _start, int _end, T _distance) :
        start(_start), end(_end), distance(_distance) {}
};

template <class T>
using Graph = std::vector<std::vector<Edge<T>>>;

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
class SmallCaseSolver {
  private:
    Graph<T> &graph;
    int node_num;
    std::vector<std::vector<T>> dp_table;
    std::vector<std::vector<int>> previous_transition;
    std::vector<int> path;
  
  public:
    SmallCaseSolver(Graph<T> &_graph) : graph(_graph), node_num(_graph.size()) {}

    void make_dp_table() {
        dp_table.assign(1 << node_num, std::vector<T>(node_num, -1));
        previous_transition.assign(1 << node_num, std::vector<int>(node_num, -1));

        for (int node = 0; node < node_num; node++) {
            dp_table[1 << node][node] = 0;
        }

        for (int set = 0; set < (1 << node_num); set++) {
            for (int node = 0; node < node_num; node++) {
                if (dp_table[set][node] == -1) { 
                    continue; 
                }
                for (const Edge<T> &edge : graph[node]) {
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
    }

    void reconstruct_path() {
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

         while (current_node != -1) {
            path.emplace_back(current_node);
            int previous_node = previous_transition[current_set][current_node];
            current_set ^= (1 << current_node);
            current_node = previous_node;
        }
    }

    std::vector<int> solve() {
        make_dp_table();
        reconstruct_path();
        return path;
    }
};

class RandomNumberGenerator {
  private:
    std::random_device seed_gen;
    std::mt19937_64 engine;

  public:
    RandomNumberGenerator() : engine(seed_gen()) {}
    
    template <class T>
    T operator()(T a, T b) {
        std::uniform_int_distribution<T> dist(a, b - 1);
        return dist(engine);
    }
    
    long double operator()() {
        std::uniform_real_distribution<long double> dist;
        return dist(engine);
    }
} rng;

class Timer {
  private:
    std::chrono::high_resolution_clock::time_point start, end;
  
  public:
    Timer() = default;

    void reset() {
        start = std::chrono::high_resolution_clock::now();
    }

    std::chrono::milliseconds::rep elapsed() {
        end = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    }
} timer;

template <class T>
class PathMaker {
  public:
    Graph<T> &graph;

    std::vector<bool> visited;
    std::vector<bool> used;

    std::vector<int> path;
    std::vector<T> edge_distance_list;

    std::vector<int> best_path;
    std::vector<T> best_edge_distance_list;

    T distance = 0;
    T longest_distance = 0;

    int start, goal;
    int remaining_search_cnt;

    PathMaker(Graph<T> &_graph, std::vector<bool> &_visited, 
                int _start, int _goal, int _remaining_search_cnt) : 
    graph(_graph), visited(_visited), 
    start(_start), goal(_goal), remaining_search_cnt(_remaining_search_cnt) {}

    void make_path_greedy(int node) {
        while (true) {
            if (visited[node] == false) {
                best_path.emplace_back(node);
                visited[node] = true;
            }

            int next_node = -1;
            T next_edge_distance = 0;
            for (const Edge<T> &edge : graph[node]) {
                if (visited[edge.end] == true) continue;
                if (ChangeMax(next_edge_distance, edge.distance)) {
                    next_node = edge.end;
                }
            }

            if (next_node == -1) break;
            
            node = next_node;
            best_edge_distance_list.emplace_back(next_edge_distance);
        }
    }

    void deapth_first_search(int node) {
        if (remaining_search_cnt == 0) return;
        remaining_search_cnt--;
        
        used[node] = true;
        path.emplace_back(node);
        
        if (node == goal) {
            if (ChangeMax(longest_distance, distance)) {
                best_path = path;
                best_edge_distance_list = edge_distance_list;
            }
        }

        for (const Edge<T> &edge : graph[node]) {
            if (remaining_search_cnt == 0) break;
            
            int next_node = edge.end;
            if (used[next_node] == true) continue;
            if (visited[next_node] == true && next_node != goal) continue;

            edge_distance_list.emplace_back(edge.distance);
            distance += edge.distance;
            
            deapth_first_search(next_node);
            
            edge_distance_list.pop_back();
            distance -= edge.distance;
        }

        path.pop_back();
        used[node] = false;
    }

    void run() {
        if (start == -1) {
            make_path_greedy(goal);
        } else {
            used.resize(graph.size(), false);
            deapth_first_search(start);
        }
    }
};

const int kTimeLimit        = 1950;
const int kStartTemperature = 150;
const int kEndTemperature   = 0;

template <class T>
class LargeCaseSolver {
  public:
    Graph<T> &graph;
    std::vector<int> path;
    std::vector<int> best_path;
    T distance = 0;
    T longest_distance = 0;
    std::vector<bool> visited;
    std::vector<T> edge_distance_list;

    LargeCaseSolver(Graph<T> &_graph) : graph(_graph), visited(_graph.size(), false) {}

    void make_initial_answer(int node) {
        while (true) {
            path.emplace_back(node);
            visited[node] = true;

            int next_node = -1;
            T next_edge_distance = 0;
            for (const Edge<T> &edge : graph[node]) {
                if (visited[edge.end] == true) {
                    continue;
                }
                if (ChangeMax(next_edge_distance, edge.distance)) {
                    next_node = edge.end;
                }
            }

            if (next_node == -1) break;

            node = next_node;
            distance += next_edge_distance;
            edge_distance_list.emplace_back(next_edge_distance);
        }
        return;
    }

    void simulated_annealing() {
        timer.reset();

        while (true) {
            if (timer.elapsed() >= kTimeLimit) {
                break;
            }

            long double temperature = 
                        kStartTemperature +
                        (kEndTemperature - kStartTemperature) * (timer.elapsed() / kTimeLimit);

            if (path.size() <= 2) {
                for (int i = 0; i < (int)graph.size(); i++) {
                    visited[i] = false;
                }
                path.clear();
                distance = 0;
                
                int start_node = rng(1, (int)graph.size());
                make_initial_answer(start_node);

                if (ChangeMax(longest_distance, distance)) {
                    best_path = path;
                }
                
                continue;
            }

            int delete_path_length = rng(1, (int)path.size() / 2 + 1);
            int start_path_id = rng(0, (int)path.size() - delete_path_length - 1);
            int end_path_id = start_path_id + delete_path_length;

            if (delete_path_length >= (int)path.size()) {
                for (int i = 0; i < (int)graph.size(); i++) {
                    visited[i] = false;
                }
                path.clear();
                distance = 0;
                
                int start_node = rng(1, (int)graph.size());
                make_initial_answer(start_node);

                if (ChangeMax(longest_distance, distance)) {
                    best_path = path;
                }
                
                continue;
            }

            T new_distance = distance;
            if (start_path_id > 0) new_distance -= edge_distance_list[start_path_id - 1];
            for (int i = start_path_id; i <= end_path_id; i++) {
                visited[path[i]] = false;
                new_distance -= edge_distance_list[i];
            }

            int start_node = (start_path_id == 0 ? -1 : path[start_path_id - 1]);
            int goal_node = path[end_path_id + 1];
            
            PathMaker<long double> path_maker(graph, visited, 
                                            start_node, goal_node,
                                            delete_path_length * 5);
            path_maker.run();

            for (int i = 0; i < path_maker.best_edge_distance_list.size(); i++) {
                new_distance += path_maker.best_edge_distance_list[i];
            }

            T diff = new_distance - distance;
            if (exp(diff / temperature) > rng()) {
                std::vector<int> new_path;
                std::vector<int> &path_add = path_maker.best_path;
                std::vector<T> new_edge_distance_list;
                std::vector<T> &edge_distance_add = path_maker.best_edge_distance_list;
                if (start_node != -1) {
                    for (int i = 0; i < start_path_id; i++) {
                        new_path.emplace_back(path[i]);
                    }
                    for (int i = 0; i < path_add.size(); i++) { 
                        if (path_add[i] != start_node && path_add[i] != goal_node) {
                            new_path.emplace_back(path_add[i]);
                            visited[path_add[i]] = true;
                        }
                    }
                    for (int i = end_path_id + 1; i < path.size(); i++) {
                        new_path.emplace_back(path[i]);
                    }

                    for (int i = 0; i < start_path_id - 1; i++) {
                        new_edge_distance_list.emplace_back(edge_distance_list[i]);
                    }
                    for (int i = 0; i < edge_distance_add.size(); i++) {
                        new_edge_distance_list.emplace_back(edge_distance_add[i]);
                    }
                    for (int i = end_path_id + 1; i < edge_distance_list.size(); i++) {
                        new_edge_distance_list.emplace_back(edge_distance_list[i]);
                    }
                } else {
                    std::reverse(path_add.begin(), path_add.end());
                    for (int i = 0; i < path_add.size(); i++) { 
                        if (path_add[i] != start_node && path_add[i] != goal_node) {
                            new_path.emplace_back(path_add[i]);
                            visited[path_add[i]] = true;
                        }
                    }
                    for (int i = end_path_id + 1; i < path.size(); i++) {
                        new_path.emplace_back(path[i]);
                    }

                    std::reverse(edge_distance_add.begin(), edge_distance_add.end());
                    for (int i = 0; i < edge_distance_add.size(); i++) {
                        new_edge_distance_list.emplace_back(edge_distance_add[i]);
                    }
                    for (int i = end_path_id + 1; i < edge_distance_list.size(); i++) {
                        new_edge_distance_list.emplace_back(edge_distance_list[i]);
                    }
                }
                path = new_path;
                edge_distance_list = new_edge_distance_list;
                distance = new_distance;
                if (ChangeMax(longest_distance, new_distance)) {
                    best_path = path;
                }
            } else {
                for (int i = start_path_id; i <= end_path_id; i++) {
                    visited[path[i]] = true;
                }
            }
        }
    }

    std::vector<int> solve() {
        make_initial_answer(0);
        best_path = path;
        longest_distance = distance;

        simulated_annealing();

        return best_path;
    }
};

template <class T>
class Compressor {
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

    int get_compressed_id(const T &x) const {
        int id = std::lower_bound(data.begin(), data.end(), x) - data.begin();
        assert(data[id] == x);
        return id;
    }

    const T &operator[](int k) const {
        assert(0 <= k && k < size());
        return data[k];
    }
};

template <class T>
class Solver {
  private:
    std::vector<Edge<T>> edges;
    Compressor<int> id_compressor;
    int node_num;
    Graph<T> graph;
    std::vector<int> route;

    void read_edges() {
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
    }

    void compress_node_id() {
        for (const Edge<T> &edge : edges) {
            id_compressor.add(edge.start);
            id_compressor.add(edge.end);
        }
        id_compressor.build();
    }

    void build_graph() {
        graph.resize(node_num);

        for (const Edge<T> &edge : edges) {
            int start = id_compressor.get_compressed_id(edge.start);
            int end = id_compressor.get_compressed_id(edge.end);
            T distance = edge.distance;

            Edge<T> new_edge(start, end, distance);
            graph[start].emplace_back(new_edge);
        }
    }

  public:
    Solver() = default;

    void init() {
        read_edges();
        compress_node_id();
        node_num = id_compressor.size();
        build_graph();
    }

    void solve() {
        if (node_num <= 16) {
            SmallCaseSolver<long double> small_case_solver(graph);
            route = small_case_solver.solve();
        } else {
            LargeCaseSolver<long double> large_case_solver(graph);
            route = large_case_solver.solve();
        }

        for (int &node : route) {
            node = id_compressor[node];
        }
    }

    void output() {
        for (int node : route) {
            std::cout << node << " ";
        }
        std::cout << "\n";
    }
};

int main() {
    Solver<long double> solver;
    solver.init();
    solver.solve();
    solver.output();
    return 0;
}
