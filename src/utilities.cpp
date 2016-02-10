#include <cstdarg>
#include <cassert>
#include <random>
#include <chrono>
#include <set>
#include <vector>
#include <iterator>

#include "utilities.h"

using namespace std;
using namespace Eigen;

std::string s_printf(const char *fmt, ...)
{
    string s;

    va_list args;

    va_start(args, fmt);
    int n = vsnprintf(0, 0, fmt, args) + 1;
    va_end(args);

    s.resize(n, 'x');

    va_start(args, fmt);
    vsnprintf(const_cast<char*>(s.data()), n, fmt, args);
    va_end(args);

    return s;
}

static std::default_random_engine g_rnd_generator;
static bool g_rnd_generator_initialized = false;

void initRandomNumberGenerators(long long seed)
{
    if (seed < 0) {
        using namespace std::chrono;
        auto T = system_clock::now().time_since_epoch();
        seed = static_cast<long long>(duration_cast<milliseconds>(T).count());
    }

    srand(seed);
    g_rnd_generator.seed(seed);

    g_rnd_generator_initialized = true;
}

double randu(double low, double high)
{
    assert(g_rnd_generator_initialized);
    return low + ((double)rand() / RAND_MAX) * (high - low);
}

double randn(double m, double sigma)
{
    assert(g_rnd_generator_initialized);
    std::normal_distribution<double> rn(m, sigma);
    return rn(g_rnd_generator);
}

std::vector<uint> getUniqueRandomInts(int minval, int maxval, size_t n)
{
    set<uint> v;
    vector<uint> w;

    while (v.size() < n) {
        v.insert(randu(minval, maxval));
    }
    copy(begin(v), end(v), back_inserter(w));

    return w;
}

void getDisjointRandomIndexSets(uint n1, uint n2, uint N, std::set<uint>& set1, std::set<uint>& set2)
{
    while (set1.size() < n1) {
        set1.insert((uint)randu(0, N - 1));
    }

    while (set2.size() < n2) {
        uint i = (uint)randu(0, N - 1);
        if (set1.find(i) != set1.end()) continue; // Skip points in set1
        set2.insert(i);
    }
}

