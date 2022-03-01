#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <nlohmann/json.hpp>

#include "utils.h"

#include "thread_pool.h"

double get_cumulative_probability(double x, double y, double h,
                                  Norm x_dist, Norm y_dist, Norm h_dist) {
    return x_dist.pdf(x) * y_dist.pdf(y) * h_dist.pdf(h);
}

double get_transition_probability(std::vector<double> const & p, std::vector<double> const & q, std::vector<double> const & delta, nlohmann::json const & mot_params) {
    auto dist_l = Norm(mot_params["linear"]["mean"], mot_params["linear"]["stddev"]);
    auto dist_a = Norm(mot_params["angular"]["mean"], mot_params["angular"]["stddev"]);

    auto s = std::sqrt(std::pow(delta[0], 2) + std::pow(delta[1], 2));
    auto s_t = std::sqrt(std::pow((p[0] - q[0]), 2) + std::pow((p[1] - q[1]), 2));

    double x;
    if (s == 0) {
        x = 1;
    } else {
        double p_l = (s_t - s) / s;
        x = dist_l.pdf(p_l);
    }
    auto p_a = normalize_angle(q[2] - p[2] - delta[2]);
    auto y = dist_a.pdf(p_a);
    return x * y;
}

int main(int argc, char ** argv) {
    if (argc < 4) {
        std::cerr << "program_name (1) - Generate trajectory with Viterbi algorithm\n"
                  << "graph (2) - path to particle transition graph files\n"
                  << "config (3) - localization config name\n"
                  << "out (4) - path to output file" << std::endl;
        return EXIT_SUCCESS;
    }
    std::string graph(argv[1]);
    std::string config(argv[2]);
    std::string out(argv[3]);


    std::fstream f_config (config, std::ios::in | std::ios::binary);
    if (!f_config.is_open()) {
        std::cerr << "can't open config" << std::endl;
        return EXIT_FAILURE;
    }
    nlohmann::json json_config;
    f_config >> json_config;
    auto x_mu = json_config["initial_position"][0];
    auto y_mu = json_config["initial_position"][1];
    auto h_mu = json_config["initial_heading"];
    auto x_dist = Norm(x_mu, json_config["init_position_stddev"]);
    auto y_dist = Norm(y_mu, json_config["init_position_stddev"]);
    auto h_dist = Norm(h_mu, json_config["init_heading_stddev"]);
    auto mot_params = json_config["motion_params"];
    auto get_prob = [&] (auto x, auto y, auto h) { return get_cumulative_probability(x, y, h, x_dist, y_dist, h_dist); };

    std::vector<nlohmann::json> json_graph;
    std::fstream f_graph (graph, std::ios::in | std::ios::binary);
    if (!f_graph.is_open()) {
        std::cerr << "can't open graph" << std::endl;
        return EXIT_FAILURE;
    }
    for (std::string line; std::getline(f_graph, line);) {
        json_graph.push_back(nlohmann::json::parse(line));
    }
    auto total_steps = json_graph.size();
    std::vector<std::vector<long long int>> prev {std::vector<long long int>(json_graph[0]["particles"]["x"].size(), -1)};
    std::vector<std::vector<double>> prob (1);

    size_t min_size = 0;
    {
        std::vector<size_t> sizes;
        sizes.push_back(json_graph[0]["particles"]["x"].size());
        sizes.push_back(json_graph[0]["particles"]["y"].size());
        sizes.push_back(json_graph[0]["particles"]["heading"].size());
        sizes.push_back(json_graph[0]["particles"]["weight"].size());
        min_size = *std::min_element(sizes.begin(), sizes.end());
    }

    for (size_t i = 0; i < min_size; i++){
        prob[0].push_back(get_prob(json_graph[0]["particles"]["x"][i],
                                   json_graph[0]["particles"]["y"][i],
                                   json_graph[0]["particles"]["heading"][i]) * static_cast<double>(json_graph[0]["particles"]["weight"][i]));
    }

    auto start = std::chrono::steady_clock::now();
    for (size_t step = 1; step < total_steps; step++) {
        auto data_cur = json_graph[step]["particles"];
        auto data_prev = json_graph[step - 1]["particles"];
        auto delta_x = json_graph[step]["delta_odometry"]["position"]["x"];
        auto delta_y = json_graph[step]["delta_odometry"]["position"]["y"];
        auto delta_h = json_graph[step]["delta_odometry"]["heading"];

        prev.emplace_back(static_cast<size_t>(data_cur["x"].size()), -1);
        prob.emplace_back(static_cast<size_t>(data_cur["x"].size()), 0);

        {
            std::vector<size_t> sizes;
            sizes.push_back(data_cur["x"].size());
            sizes.push_back(data_cur["y"].size());
            sizes.push_back(data_cur["heading"].size());
            sizes.push_back(data_cur["weight"].size());
            min_size = *std::min_element(sizes.begin(), sizes.end());
        }

        {
            ThreadPool thread_pool(std::max(static_cast<unsigned int>(2), std::thread::hardware_concurrency()));
            for (size_t i = 0; i < min_size; i++) {
                size_t min_size_2;
                {
                    std::vector<size_t> sizes;
                    sizes.push_back(data_prev["x"].size());
                    sizes.push_back(data_prev["y"].size());
                    sizes.push_back(data_prev["heading"].size());
                    min_size_2 = *std::min_element(sizes.begin(), sizes.end());
                }

                thread_pool.AddTask(
                        [i, step, min_size_2, &prob, &prev, &data_cur, &data_prev, &delta_x, &delta_y, &delta_h, &mot_params] {
                            for (size_t j = 0; j < min_size_2; j++) {
                                auto tran_prob = get_transition_probability(
                                        {
                                                static_cast<double>(data_prev["x"][j]),
                                                static_cast<double>(data_prev["y"][j]),
                                                static_cast<double>(data_prev["heading"][j])
                                        },
                                        {
                                                static_cast<double>(data_cur["x"][i]),
                                                static_cast<double>(data_cur["y"][i]),
                                                static_cast<double>(data_cur["heading"][i])
                                        },
                                        {
                                                delta_x,
                                                delta_y,
                                                delta_h
                                        },
                                        mot_params
                                );
                                if (prob[step][i] <
                                    static_cast<double>(data_cur["weight"][i]) * tran_prob * prob[step - 1][j]) {
                                    prob[step][i] = static_cast<double>(data_cur["weight"][i]) * tran_prob
                                                    * prob[step - 1][j];
                                    prev[step][i] = j;
                                }
                            }
                        });
            }
        }

        double s = std::accumulate(prob[step].begin(), prob[step].end(), static_cast<double>(0));
        for (auto &x: prob[step]) {
            x /= s;
        }
    }
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() << " ms" << std::endl;

    int j = argmax(prob.back());
    std::vector<size_t> idx_path;
    auto i = json_graph.size() - 1;
    while (j != -1) {
        idx_path.push_back(j);
        j = prev[i][j];
        i -= 1;
    }
    nlohmann::json trajectory;
    std::reverse(idx_path.begin(), idx_path.end());
    for (size_t p = 0; p < idx_path.size(); ++p) {
        nlohmann::json temp_vec;
        temp_vec.push_back(json_graph[p]["particles"]["x"][idx_path[p]]);
        temp_vec.push_back(json_graph[p]["particles"]["y"][idx_path[p]]);
        temp_vec.push_back(json_graph[p]["particles"]["heading"][idx_path[p]]);

        trajectory.push_back(temp_vec);
    }

    std::fstream f_out(out, std::ios::out | std::ios::binary);
    if (!f_out.is_open()) {
        std::cerr << "can't create out" << std::endl;
        return EXIT_FAILURE;
    }
    f_out << trajectory;

    return EXIT_SUCCESS;
}
