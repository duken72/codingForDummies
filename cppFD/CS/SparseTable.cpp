#include <iostream>
#include <array>
#include <vector>
#include <algorithm>

template<typename T = int>
class SparseTable
{
    const int m_col, m_row;
    std::vector<std::vector<T>> m_dp;
    std::vector<T> m_lookup_log2;

    static T f(T x, T y) { return std::min(x, y); };
    bool overlap_agnostic = true;   // based on the function f

    static int log2(int x)
    {
        int ans = 0;
        while (x >>= 1)
            ans++;
        return ans;
    };

    static int pow2(int x) {
        return (1 << x);
    }

public:
    SparseTable(const std::vector<T>& vectIn)
    : m_col(vectIn.size()), m_row(log2(m_col) + 1),
    m_dp(m_row, std::vector<T>(m_col, 0)), m_lookup_log2(m_col, 0)
    {
        // Fill first row
        for (int i = 0; i < m_col; i++)
            m_dp[0][i] = vectIn[i];

        // Fill other rows
        for (int i = 1; i < m_row; i++) {
            for (int j = 0; (j + (1 << i) - 1) < m_col; j++) {
                m_dp[i][j] = f(m_dp[i-1][j], m_dp[i-1][j + pow2(i - 1)]);
            }
        }
    }

    T query(int l, int r)
    {
        int p = log2(r - l + 1);
        // If overlap agnostic, O(1)
        if (overlap_agnostic)
            return f(m_dp[p][l], m_dp[p][r - (1 << p) + 1]);
        // If not overlap agnostic, O(log2 N)
        T ans = 1e9;
        for (p = log2(r - l + 1); l <= r; p = log2(r - l + 1)) {
            ans = f(ans, m_dp[p][l]);
            l += (1 << p);
        }
        return ans;
    };

    void print()
    {
        std::cout << "Sparse table:\n";
        for (int i = 0; i < m_row; i++) {
            for (int j = 0; j < m_col; j++) {
                std::cout << m_dp[i][j] << " ";
            }
            std::cout << "\n";
        }
        std::cout << "\n";
    };
};

int main(int argc, char const *argv[])
{
    std::vector<int> vi = { 4, 2, 3, 7, 1, 5, 3, 3, 9, 6, 7, -1, 4 };
    SparseTable<int> st(vi);
    st.print();
    std::cout << st.query(1, 11) << std::endl;
    std::cout << st.query(2, 7) << std::endl;
    return 0;
}
