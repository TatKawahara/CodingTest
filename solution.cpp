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

// 辺のクラス
template <class T>
class Edge {
  public:
    // 頂点 start から頂点 end へ張られた、距離 distance の辺
    int start, end;
    T distance;

    Edge() = default;
    Edge(int _start, int _end, T _distance) :
        start(_start), end(_end), distance(_distance) {}
};

// グラフの型エイリアス
template <class T>
using Graph = std::vector<std::vector<Edge<T>>>;

// 最大値更新の処理を簡単に行うための関数
template <class T>
bool ChangeMax(T &current_value, const T &new_value) {
    if (current_value < new_value) {
        current_value = new_value;
        return true;
    } else {
        return false;
    }
}

// 頂点数が少ない場合のソルバー
template <class T>
class SmallCaseSolver {
  private:
    // node_num はグラフの頂点数
    // previous_transition はどこから遷移したのかを管理する配列
    //      例えば previous_transition[next_s][next_v] = v のとき、
    //      dp_table[next_s][next_v] を達成するには
    //      「今まで通った頂点の集合が next_s xor (2 ^ next_v) で、頂点 v にいる」という状態から
    //      v - next_v を結ぶ辺を通るとよい
    Graph<T> &graph;
    int node_num;
    std::vector<std::vector<T>> dp_table;
    std::vector<std::vector<int>> previous_transition;
    std::vector<int> path;
  
  public:
    SmallCaseSolver(Graph<T> &_graph) : graph(_graph), node_num(_graph.size()) {}

    void make_dp_table() {
        // 最大値を更新していくので、最初は小さい値にしておく
        dp_table.assign(1 << node_num, std::vector<T>(node_num, -1));
        previous_transition.assign(1 << node_num, std::vector<int>(node_num, -1));

        // スタート時は各頂点 v について「今まで通った頂点が v のみで、現在 v にいる」状態
        // スタート時、辺はまだ 1 本も通っていないので、かかる距離は 0
        for (int node = 0; node < node_num; node++) {
            dp_table[1 << node][node] = 0;
        }

        // set の小さい順に dp_table を更新していく
        for (int set = 0; set < (1 << node_num); set++) {
            for (int node = 0; node < node_num; node++) {
                if (dp_table[set][node] == -1) { 
                    continue; 
                }
                for (const Edge<T> &edge : graph[node]) {
                    int next_node = edge.end;
                    T distance = edge.distance;
                    
                    // next_node が既に訪問済みの場合は飛ばす
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

    // 経路の復元
    void reconstruct_path() {
        // dp_table[S][v] = (dp_table の最大値) となる S と v を求める
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

        // 遡っていく
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

// 乱数生成器
class RandomNumberGenerator {
  private:
    std::random_device seed_gen;    // シード値生成用
    std::mt19937_64 engine;         // 擬似乱数生成器メルセンヌ・ツイスタ

  public:
    RandomNumberGenerator() : engine(seed_gen()) {} // シード値 seed_gen() で初期化
    
    // a 以上 b 未満の整数をランダムに生成
    template <class T>
    T generate_integer(T a, T b) {
        std::uniform_int_distribution<T> dist(a, b - 1);
        return dist(engine);
    }
    
    // 0 以上 1.0 未満の小数をランダムに生成
    long double generate_decimal() {
        std::uniform_real_distribution<long double> dist;
        return dist(engine);
    }
} rng;

// 時間計測
class Timer {
  private:
    std::chrono::high_resolution_clock::time_point start, end;
  
  public:
    Timer() = default;

    // 計測開始
    void reset() {
        start = std::chrono::high_resolution_clock::now();
    }

    // 計測を開始してから現在までの経過時間 (単位はミリ秒)
    std::chrono::milliseconds::rep elapsed() {
        end = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    }
} timer;

// 焼きなまし法で近傍 (暫定解のパスを少し変えたパス) を作る際に用いるクラス
template <class T>
class PathMaker {
  public:
    Graph<T> &graph;
    std::vector<bool> visited;              // 既にパスに含まれているかどうか
    std::vector<bool> used;                 // DFS 時に探索済みかどうか
    std::vector<int> path;                  // 現在までに通った頂点のリスト
    std::vector<T> edge_distance_list;      // 現在までに通った辺の長さのリスト
    std::vector<int> best_path;             // 最長経路の頂点のリスト
    std::vector<T> best_edge_distance_list; // 最長経路の辺の長さのリスト
    T distance = 0;
    T longest_distance = 0;
    int start, goal;
    int remaining_search_cnt;               // 残り探索回数

    PathMaker(Graph<T> &_graph, std::vector<bool> &_visited, 
                int _start, int _goal, int _remaining_search_cnt) : 
    graph(_graph), visited(_visited), 
    start(_start), goal(_goal), remaining_search_cnt(_remaining_search_cnt) {}

    // 頂点 node からスタートして、「次行ける辺のうち最長のものを通る」を繰り返す
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

            // 次に行ける頂点がなくなったら終了
            if (next_node == -1) break;
            
            node = next_node;
            best_edge_distance_list.emplace_back(next_edge_distance);
        }
    }

    // 深さ優先探索
    void deapth_first_search(int node) {
        if (remaining_search_cnt == 0) return;  // 残り探索回数が 0 になったら終了
        remaining_search_cnt--;
        
        used[node] = true;
        path.emplace_back(node);
        
        // goal に到達した場合
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
            
            // 探索が終わったら、未訪問にする (バックトラック)
            edge_distance_list.pop_back();
            distance -= edge.distance;
        }

        // 探索が終わったら、未訪問にする (バックトラック)
        path.pop_back();
        used[node] = false;
    }

    // 実行
    void run() {
        if (goal == -1) {
            // 選ばれた区間がもとの経路の端点を含んでいる場合
            // 頂点 start から貪欲にパスを伸ばしていく
            make_path_greedy(start);
        } else {
            // 暫定解の経路が 2 つに分裂した場合
            // 頂点 start と頂点 goal を結ぶパスを作る
            used.resize(graph.size(), false);
            deapth_first_search(start);
        }
    }
};

 // 実行時間制限
const int kTimeLimit = 1950;

// 焼きなまし法の温度計算に使う
const int kStartTemperature = 150;
const int kEndTemperature   = 0;    


// 頂点数が多い場合のソルバー
template <class T>
class LargeCaseSolver {
  public:
    Graph<T> &graph;
    std::vector<int> path;              // 暫定解の経路の頂点のリスト
    std::vector<int> best_path;         // 最長経路の頂点のリスト
    T distance = 0;
    T longest_distance = 0;
    std::vector<bool> visited;          // 頂点が訪問済みかどうかを管理
    std::vector<T> edge_distance_list;  // 暫定解の経路の、辺の長さのリスト

    LargeCaseSolver(Graph<T> &_graph) : graph(_graph), visited(_graph.size(), false) {}

    // 「次に行ける辺のうち、最長のもの」を選ぶ貪欲法
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

    // 焼きなまし法
    void simulated_annealing() {
        timer.reset();

        while (true) {
            // 実行時間制限を過ぎたら終了
            if (timer.elapsed() >= kTimeLimit) {
                break;
            }

            // 温度の計算
            long double temperature = 
                        kStartTemperature +
                        (kEndTemperature - kStartTemperature) * (timer.elapsed() / kTimeLimit);

            // パスの長さが短い場合は、暫定解を一から作り直す
            // 頂点 0 以外の頂点をランダムに選び、例の貪欲法
            if (path.size() <= 2) {
                for (int i = 0; i < (int)graph.size(); i++) {
                    visited[i] = false;
                }
                path.clear();
                distance = 0;
                
                int start_node = rng.generate_integer(1, (int)graph.size());
                make_initial_answer(start_node);

                if (ChangeMax(longest_distance, distance)) {
                    best_path = path;
                }
                
                continue;
            }

            // 削除する区間の長さ、始点、終点を定める
            int delete_path_length = rng.generate_integer(1, (int)path.size() / 2 + 1);
            int start_path_id = rng.generate_integer(0, (int)path.size() - delete_path_length - 1);
            int end_path_id = start_path_id + delete_path_length;

            // new_distance は新しい経路 (近傍) の長さ
            // 選んだ区間を経路から削除する (陽に削除操作をするわけではない)
            // ・削除される頂点を未訪問にする
            // ・削除される辺の長さを new_distance から引く
            T new_distance = distance;
            if (start_path_id > 0) new_distance -= edge_distance_list[start_path_id - 1];
            for (int i = start_path_id; i <= end_path_id; i++) {
                visited[path[i]] = false;
                new_distance -= edge_distance_list[i];
            }

            // 繋ぐ経路の端点
            // 選ばれた区間がもとの経路の端点を含んでいる場合は、goal_node = -1 にしておく
            int start_node = (start_path_id == 0 ? path.back() : path[start_path_id - 1]);
            int goal_node  = (start_path_id == 0 ?      -1     : path[end_path_id + 1]);
            
            // 近傍を作るための経路を探索
            PathMaker<long double> path_maker(graph, visited, 
                                            start_node, goal_node,
                                            delete_path_length * 5);
            path_maker.run();

            // 繋ぐ経路が見つからなかった場合は、暫定解の変更を行わない
            if (path_maker.best_path.size() == 0) {
                for (int i = start_path_id; i <= end_path_id; i++) {
                    visited[path[i]] = true;
                }
                continue;
            }
            
            // 繋ぐ経路の長さを new_distance に足す
            std::vector<T> &edge_distance_add = path_maker.best_edge_distance_list;
            for (int i = 0; i < edge_distance_add.size(); i++) {
                new_distance += edge_distance_add[i];
            }

            // 近傍を採用するかどうか判定
            T difference = new_distance - distance;
            if (exp(difference / temperature) > rng.generate_decimal()) {
                // 採用する場合
                std::vector<int> new_path;
                std::vector<int> &path_add = path_maker.best_path;
                std::vector<T> new_edge_distance_list;
                if (goal_node != -1) {
                    // 分裂した経路を繋ぐ場合
                    // 3 つのループを回す
                    // ・分裂された経路の左側、新たに繋ぐ経路、分裂された経路の右側の順
                    
                    // 新しい経路の、頂点のリストを作る
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

                    // 新しい経路の、辺の長さのリストを作る
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
                    // 端点から伸ばす場合
                    
                    // 新しい経路の、頂点のリストを作る
                    for (int i = end_path_id + 1; i < path.size(); i++) {
                        new_path.emplace_back(path[i]);
                    }
                    for (int i = 0; i < path_add.size(); i++) { 
                        if (path_add[i] != start_node && path_add[i] != goal_node) {
                            new_path.emplace_back(path_add[i]);
                            visited[path_add[i]] = true;
                        }
                    }

                    // 新しい経路の、辺の長さのリストを作る
                    for (int i = end_path_id + 1; i < edge_distance_list.size(); i++) {
                        new_edge_distance_list.emplace_back(edge_distance_list[i]);
                    }
                    for (int i = 0; i < edge_distance_add.size(); i++) {
                        new_edge_distance_list.emplace_back(edge_distance_add[i]);
                    }
                }
                // 暫定解を近傍に更新
                path = new_path;
                edge_distance_list = new_edge_distance_list;
                distance = new_distance;

                // 最長距離を更新
                if (ChangeMax(longest_distance, new_distance)) {
                    best_path = path;
                }
            } else {
                // 採用しない場合、もとに戻す
                for (int i = start_path_id; i <= end_path_id; i++) {
                    visited[path[i]] = true;
                }
            }
        }
    }

    std::vector<int> solve() {
        make_initial_answer(0);         // 初期解を構築
        best_path = path;
        longest_distance = distance;

        simulated_annealing();          // 焼きなまし法

        return best_path;
    }
};

// 座標圧縮をするクラス
template <class T>
class Compressor {
  private:
    std::vector<T> data;

  public:
    Compressor() = default;
    
    // 登場する値を追加
    void add(const T &x) {
        data.emplace_back(x);
    }

    // 登場する値を小さい順にソートし、重複する要素を削除する
    void build() {
        std::sort(data.begin(), data.end());
        data.erase(std::unique(data.begin(), data.end()), data.end());
    }

    size_t size() const { return data.size(); }

    // 圧縮後の値を返す
    // つまり、その値が登場する値の中で何番目に小さいかを返す
    int get_compressed_id(const T &x) const {
        int id = std::lower_bound(data.begin(), data.end(), x) - data.begin();
        assert(data[id] == x);
        return id;
    }

    // 圧縮後の値を引数として、圧縮前のもとの値を返す
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

    // 入力を受け取り、辺のリストを作る
    void read_edges() {
        // 一行ずつ受け取り、「始点のID」「コンマ」「終点のID」「コンマ」「長さ」に分ける
        std::string line;
        while (std::getline(std::cin, line)) {
            std::stringstream ss(line);
            
            int start, end;
            long double distance;
            char comma;

            ss >> start >> comma >> end >> comma >> distance;
            
            // start から end へ向かう辺を追加
            Edge<T> edge(start, end, distance);
            edges.emplace_back(edge);

            // end から start へ向かう辺を追加
            Edge<T> rev_edge(end, start, distance);
            edges.emplace_back(rev_edge);
        }
    }

    // 座標圧縮のインスタンスを作る
    void compress_node_id() {
        for (const Edge<T> &edge : edges) {
            id_compressor.add(edge.start);
            id_compressor.add(edge.end);
        }
        id_compressor.build();
    }

    // 辺のリストと座標圧縮のインスタンスからグラフを作る
    void build_graph() {
        graph.resize(node_num);

        for (const Edge<T> &edge : edges) {
            // グラフ内では ID を座標圧縮した値を持つ
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
        
        // 座標圧縮前の ID に戻す
        for (int &node : route) {
            node = id_compressor[node];
        }
    }

    void output() {
        for (int node : route) {
            std::cout << node << "\r\n";
        }
    }
};

int main() {
    Solver<long double> solver;
    solver.init();
    solver.solve();
    solver.output();
    return 0;
}
