// https://en.cppreference.com/w/cpp/algorithm/sort

#include <algorithm>
#include <functional>
#include <array>
#include <iostream>
#include <string_view>
 
int main()
{
    std::cout << "Sorting array:\n";
    std::array<int, 10> s = {5, 7, 4, 2, 8, 6, 1, 9, 0, 3};
 
    auto print = [&s](std::string_view const rem)
    {
        for (auto a : s)
            std::cout << a << ' ';
        std::cout << ": " << rem << '\n';
    };
 
    std::sort(s.begin(), s.end());
    print("with default operator<");
 
    std::sort(s.begin(), s.end(), std::greater<int>());
    print("with the STL compare function object");
 
    struct
    {
        bool operator()(int a, int b) const { return a < b; }
    }
    customLess;
 
    std::sort(s.begin(), s.end(), customLess);
    print("with custom function object");
 
    std::sort(s.begin(), s.end(), [](int a, int b) { return a > b; });
    print("with lambda expression");
}
