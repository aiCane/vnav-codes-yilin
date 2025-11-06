#include "random_vector.h"
#include <cfloat>
#include <iostream>
#include <random>
#include <vector>
// #include <algorithm>
// #include <numeric>

RandomVector::RandomVector(int size, double max_val) {
    std::random_device rd;
    std::mt19937 eng(rd()); // Mersenne Twister 19937
    std::uniform_real_distribution<double> distr(0.0, max_val);
    for (int i = 0; i < size; i++) {
        vect.push_back(distr(eng));
    }
}

void RandomVector::print() {
    for (double it : vect) {
        std::cout << it << std::endl;
    }
}

double RandomVector::mean() {
    double mean_to_return = 0.0;
    for (double it : vect) {
        mean_to_return += it;
    }
    mean_to_return /= vect.size();
    return mean_to_return;
    // return std::accumulate(vect.begin(), vect.end(), 0);
}

double RandomVector::max(){
    double max_to_return = 0.0;
    for (double it : vect) {
        if (max_to_return < it) {
            max_to_return = it;
        }
    }
    return max_to_return;
    // return *std::max_element(vect.begin(), vect.end()); // just like *void in C
}

double RandomVector::min(){
    double min_to_return = DBL_MAX;
    for (double it : vect) {
        if (it < min_to_return) {
           min_to_return = it;
        }
    }
    return min_to_return;
    // return *std::min_element(vect.begin(), vect.end());
}

void RandomVector::printHistogram(int bins){
    // TODO: bins == 0 or bins == 1
    double min_value = min();
    double bin_length = (max() - min_value) / bins;
    std::vector<double> separaters, volumns;
    for (int i = 1; i < bins; i++) {
        separaters.push_back(min_value + i * bin_length);
    }
    for (double it : vect) {
        vect.pop_back();
    }
}
