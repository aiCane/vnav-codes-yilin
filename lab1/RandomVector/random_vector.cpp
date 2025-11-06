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
    for (int i = 0; i < size; i++) vect.push_back(distr(eng));
}

void RandomVector::print() {
    for (double v : vect) std::cout << v << " ";
    std::cout << std::endl;
}

double RandomVector::mean() {
    double mean_to_return = 0.0;
    for (double v : vect) mean_to_return += v;
    mean_to_return /= vect.size();
    return mean_to_return;
    // return std::accumulate(vect.begin(), vect.end(), 0);
}

double RandomVector::max(){
    double max_to_return = 0.0;
    for (double v : vect) {
        if (max_to_return < v) max_to_return = v;
    }
    return max_to_return;
    // return *std::max_element(vect.begin(), vect.end()); // just like *void in C
}

double RandomVector::min(){
    double min_to_return = DBL_MAX;
    for (double v : vect) {
        if (v < min_to_return) min_to_return = v;
    }
    return min_to_return;
    // return *std::min_element(vect.begin(), vect.end());
}

void RandomVector::printHistogram(int bins){
    // TODO: bins == 0 or bins == 1
    double min_value = min();
    double bin_length = (max() - min_value) / bins;
    std::vector<double> separaters, sorted_vect = vect;
    std::vector<int> volumns;
    int temp_max, max_volumn = 0, j = 0;

    for (int i = 0; i < bins; i++) {
        volumns.push_back(0);
        min_value += bin_length;
        separaters.push_back(min_value);
    }

    mySort(sorted_vect);
    for (double v : sorted_vect) {
        if (v <= separaters.at(j)) {
            volumns.at(j)++;
            continue;
        }
        if (max_volumn < volumns.at(j)) max_volumn = volumns.at(j);
        volumns.at(++j)++;
    }

    temp_max = max_volumn;
    for (int i = 0; i < max_volumn; i++) {
        for (j = 0; j < bins; j++) {
            if (volumns.at(j) < temp_max) {
                std::cout << "    ";
                continue;
            }
            volumns.at(j)--;
            std::cout << "*** ";
        }
        temp_max--;
        std::cout << std::endl;
    }
}

void RandomVector::mySort(std::vector<double> & vect) {
	// bubble sort
	int len = vect.size();
	double temp;
	for (int i = len - 1; i > 0; --i) {
		for (int j = 0; j < i; ++j) {
			if (vect[j] > vect[j + 1]) {
				temp = vect[j];
				vect[j] = vect[j + 1];
				vect[j + 1] = temp;
			}
		}
	}
}
