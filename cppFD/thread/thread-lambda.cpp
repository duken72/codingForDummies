#include <chrono>
#include <iostream>
#include <numeric>
#include <thread>
#include <vector>

using namespace std;
using namespace std::chrono;
using std::chrono::duration_cast;
using ms = std::chrono::milliseconds;
using LL = long long;

int main() {
  auto t0 = steady_clock::now();
  const int no_threads = 1024;
  const LL no_elements = 2 * 1000 * 1000 * 1000;
  const LL step = no_elements / no_threads;
  vector<thread> threads;
  vector<LL> partial_sums(no_threads);

  for (int i = 0; i < no_threads; i++) {
    threads.push_back(thread([i, &partial_sums, step] {
      for (LL j = i * step; j < (i + 1) * step; j++)
        partial_sums[i] += j;
    }));
  }

  for (std::thread &t : threads)
    if (t.joinable())
      t.join();

  LL total = 0;
  for (int i = 0; i < no_threads; i++) {
    cout << partial_sums[i] << " ";
    total += partial_sums[i];
  }
  cout << endl;
  cout << "total: " << total << endl;
  auto t1 = steady_clock::now();
  cout << "Time t = " << duration_cast<ms>(t1 - t0).count() << "[ms]" << endl;
  return 0;
}
