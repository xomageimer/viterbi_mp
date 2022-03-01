#ifndef VITERBI_UTILS_H
#define VITERBI_UTILS_H

#include <vector>
#include <cmath>

#define M_PI 3.14159265358979323846

struct Norm {
public:
    Norm(double mu, double std) : mu_(mu), std_(std) {}
    [[nodiscard]] double pdf(double x) const {
        return std::exp(-std::pow((x - mu_), 2) / (2 * std::pow(std_, 2))) / std_ / std::sqrt((2 * M_PI));
    }
private:
    double mu_;
    double std_;
    static inline double step_ = 1e-3;
};
double normalize_angle(double angle);
size_t argmax(std::vector<double> const & v);

#endif //VITERBI_UTILS_H
