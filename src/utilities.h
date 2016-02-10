#pragma once

#include <set>
#include <vector>
#include <sstream>
#include <eigen3/Eigen/Eigen>
#include "pose.h"
#include "common.h"

/// Initialize the RNGs
void initRandomNumberGenerators(long long seed = -1);

/// Get a random number from a uniform distribution in the range (low, high(
double randu(double low, double high);

/// Get a random number from a normal distribution N(m, sigma^2)
double randn(double m, double sigma);

/// Get N unique uniformly distributed random integers in the range [minval, maxval[
std::vector<uint> getUniqueRandomInts(int minval, int maxval, size_t n);

/// Draw n1+n2 uniformly distributed unsigned integer random numbers and place them in two disjoint sets. The numbers are drawn from [0, N[
void getDisjointRandomIndexSets(uint n1, uint n2, uint N, std::set<uint>& set1, std::set<uint>& set2);

/** Extract vector elements determined by an index set
*
* @param data    Vector to extract elements from.
* @pram indices  Indices of elements to choose.
*
* @returns Extracted elements
*/
template <typename T>
static std::vector<T> selectFromIndexSet(const std::vector<T>& data, const std::set<uint>& indices)
{
    std::vector<T> selected(indices.size());

    int j = 0;
    for (auto& i : indices) {
        selected[j] = data[i];
        j++;
    }

    return selected;
}

/// Convenience macro for fprintf(stderr, ...)
#define eprintf(...) fprintf(stderr, __VA_ARGS__)

/// sprintf with std::string output
std::string s_printf(const char *fmt, ...);

/// Convert a vector of numbers to a string that can be parsed by Matlab
template <typename T>
std::string toString(const std::vector<T>& m)
{
    std::stringstream ss;

    ss << "[";
    for (size_t i = 0; i < m.size(); ++i) {
        ss << m[i];
        if (i != (m.size() - 1))
            ss << "; ";
        if ((i > 0) && (i % 50) == 0)
            ss << "\n";
    }
    ss << "];";

    return ss.str();
}

/// Convert a Pose to a string that can be parsed by Matlab
std::string toString(const Pose& p);

/// Convert a 3x3 matrix to a string that can be parsed by Matlab
std::string toString(const Eigen::Matrix3d& m);
