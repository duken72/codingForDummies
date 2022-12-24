#include <iostream>
#include <numeric>
#include <thread>
#include <vector>
#include <chrono>

using namespace std;
using namespace std::chrono;
using std::chrono::duration_cast;
using ms = std::chrono::milliseconds;
using LL = long long;

class AccumulateFunctor {
public:
    void operator()(LL start, LL end) {
        _sum = 0;
        for (auto i = start; i < end; i++)
            _sum += i;
        cout << _sum << endl;
    }
    LL _sum;
};

int main() {
    auto t0 = steady_clock::now();
    const int no_threads = 16;
    const LL no_elements = 2 * 1000 * 1000 * 1000;
    const LL step = no_elements / no_threads;
    vector<thread> threads;
    vector<AccumulateFunctor *> functors;

    for (int i = 0; i < no_threads; i++) {
        AccumulateFunctor *functor = new AccumulateFunctor();
        threads.push_back(
            thread(ref(*functor), i * step, (i + 1) * step));
        functors.push_back(functor);
    }

    for (std::thread &t : threads)
        if (t.joinable()) t.join();

    LL total = 0;
    for (auto pf : functors)
        total += pf->_sum;
    cout << "total: " << total << endl;
    auto t1 = steady_clock::now();
    cout << "Time t = "
         << duration_cast<ms>(t1 - t0).count() << "[ms]" << endl;
    return 0;
}