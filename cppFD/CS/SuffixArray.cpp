// https://codeforces.com/edu/course/2/lesson/2

#pragma GCC optimize("O2,unroll-loops")
#pragma GCC target("avx2,bmi,bmi2,lzcnt,popcnt")

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;

#define FI first
#define SE second

void count_sort(vector<int>& p, vector<int>& c)
{
    int n = p.size();
    vector<int> cnt(n);         // bucket's sizes
    for (auto x : c)    cnt[x]++;

    vector<int> p_new(n);
    vector<int> pos(n);         // each bucket iter positions
    pos[0] = 0;
    for (int i = 1; i < n; i++)     pos[i] = pos[i - 1] + cnt[i - 1];

    for (auto x : p) {
        p_new[pos[c[x]]] = x;
        pos[c[x]]++;
    }
    p = p_new;
}

vector<int> suffix_array(string& s) {
    s += "$";
    int n = s.size();
    vector<int> p(n), c(n);
    int k = 0;
    {
        vector<pair<char, int>> a(n);       // k = 0
        for(int i = 0; i < n; i++)  a[i] = {s[i], i};
        sort(a.begin(), a.end());
        for(int i = 0; i < n; i++)  p[i] = a[i].SE;

        c[p[0]] = 0;
        for(int i = 1; i < n; i++)
            c[p[i]] = c[p[i-1]] + int(a[i-1].FI != a[i].FI);
    }

    while((1 << k) < n && c[p.back()] < n-1) {
        // k -> k + 1
        // Shift to get the sorted array by second halves
        for (int i = 0; i < n; i++)
            p[i] = (p[i] - (1 << k) + n) % n;
        count_sort(p, c);       // Count sort for the 1st halves

        vector<int> c_new(n);
        c_new[p[0]] = 0;
        pair<int, int> prev = {c[p[0]], c[(p[0] + (1 << k)) % n]};
        for (int i = 1; i < n; i++) {
            pair<int, int> now = {c[p[i]], c[(p[i] + (1 << k)) % n]};
            c_new[p[i]] = c_new[p[i - 1]] + !(now == prev);
            prev = now;
        }
        c = c_new;
        k++;
    }
    return p;
}

int main(int argc, char const *argv[])
{
    string str = "banana";
    vector<int> sa = suffix_array(str);
    for (int v : sa) {
        cout << v << ' ' << str.substr(v) << '\n';
    }    
    return 0;
}
