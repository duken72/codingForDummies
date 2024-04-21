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

void AccumulateRange(LL &sum, LL start, LL end) {
  sum = 0;
  for (LL i = start; i < end; i++)
    sum += i;
}

int main() {
  auto t0 = steady_clock::now();

  const int no_threads = 256;
  const LL no_elements = 2 * 1000 * 1000 * 1000;
  const LL step = no_elements / no_threads;
  vector<thread> threads;
  LL partial_sums[no_threads];

  for (LL i = 0; i < no_threads; i++)
    threads.push_back(thread(AccumulateRange, ref(partial_sums[i]), i * step,
                             (i + 1) * step));

  for (thread &t : threads)
    if (t.joinable())
      t.join();

  LL total = 0;
  for (int i = 0; i < no_threads; i++) {
    cout << partial_sums[i] << " ";
    total += partial_sums[i];
  }
  cout << endl;

  cout << "total = " << total << endl;
  auto t1 = steady_clock::now();
  cout << "Time t = " << duration_cast<ms>(t1 - t0).count() << "[ms]" << endl;
  return 0;
}
