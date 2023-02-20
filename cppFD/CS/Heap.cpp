// References:
// https://en.cppreference.com/w/cpp/algorithm/ranges/make_heap
// https://en.cppreference.com/w/cpp/algorithm/push_heap

#include <algorithm>
#include <vector>
#include <cmath>
#include <iostream>
#include <string_view>

using namespace std;

void out(const auto& what, int n = 1) { while (n-- > 0) std::cout << what; }
void print(std::string_view text, std::vector<int> const& v = {}) {
    std::cout << text << ": ";
    for (const auto& e : v) std::cout << e << ' ';
    std::cout << '\n';
}
void draw_heap(auto const& v);

#define USING_STD_LIB true
#define USING_STD_RANGES !USING_STD_LIB

int main(int argc, char const *argv[])
{
    vector h {1, 6, 1, 8, 0, 3, 3, 9, 8, 8, 7, 4, 9, 8, 9};
    print("Ini vect", h);

#if USING_STD_LIB
    // Min heap
    make_heap(h.begin(), h.end(), std::greater<>{});
    print("Min heap", h);
    draw_heap(h);

    // Max heap
    if (!std::is_heap(h.begin(), h.end())) {
        std::cout << "\nChecking with is_heap...\n";
        make_heap(h.begin(), h.end());
    }
    print("Max heap", h);
    draw_heap(h);

    // Pop heap: swap the top elem to the last
    // then bubbling down to meet the heap invariant
    print("\nUsing pop_heap");
    pop_heap(h.begin(), h.end());
    draw_heap(h);
    h.pop_back();
    draw_heap(h);

    // Push heap: add a new elem to the heap
    print("\nUsing push_heap");
    h.push_back(14);
    draw_heap(h);
    push_heap(h.begin(), h.end());
    draw_heap(h);

    // Sort heap: 
    print("\nUsing sort_heap h", h);
    sort_heap(h.begin(), h.end());

#else   // USING_STD_RANGES
    ranges::make_heap(h);
    print("\n" "max-heap", h);
    draw_heap(h);

    ranges::make_heap(h, std::greater{});
    print("\n" "min-heap", h);
    draw_heap(h);
#endif  // USING_STD_LIB

    return 0;
}

void draw_heap(auto const& v)
{
    auto bails = [](int n, int w) {
        auto b = [](int w) { out("┌"), out("─", w), out("┴"), out("─", w), out("┐"); };
        if (!(n /= 2)) return;
        for (out(' ', w); n-- > 0; ) b(w), out(' ', w + w + 1);
        out('\n');
    };
    auto data = [](int n, int w, auto& first, auto last) {
        for(out(' ', w); n-- > 0 && first != last; ++first)
            out(*first), out(' ', w + w + 1);
        out('\n');
    };
    auto tier = [&](int t, int m, auto& first, auto last) {
        const int n {1 << t};
        const int w {(1 << (m - t - 1)) - 1};
        bails(n, w), data(n, w, first, last);
    };
    const int m {static_cast<int>(std::ceil(std::log2(1 + v.size())))};
    auto first {v.cbegin()};
    for (int i{}; i != m; ++i) { tier(i, m, first, v.cend()); }
}
