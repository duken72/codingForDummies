#include <vector>
#include <iostream>
#include <exception>

template<typename T = int>
class FenwickTree {
private:
    // The original array and the tree
    std::vector<T> m_Array, m_Tree;

    // Least Significant Bit (id & ~(id-1) also works)
    static inline T LSB(const T& i) { return i & -i; };

public:
    FenwickTree(int N) : m_Array(N + 1), m_Tree(N + 1) {};

    FenwickTree(const std::vector<T>& arr)
    {
        size_t N = arr.size() + 1;
        m_Array = std::vector<T>(N + 1);
        for (size_t i = 1; i <= N; i++)
            m_Array[i] = arr[i - 1];

        m_Tree = m_Array;
        for (size_t i = 1, j; i <= N; i++) {
            j = i + LSB(i);
            if (j <= N)     m_Tree[j] += m_Tree[i];
        }
    };

    // Get array value
    T at(int id) const
    {
        if (id <= 0)
            throw std::invalid_argument("id has to be positive: id > 0");
        return m_Array[id];
    }

    // ID query
    T query(int id) const
    {
        if (id <= 0)
            throw std::invalid_argument("id has to be positive: id > 0");
        T sum = 0;
        for (; id > 0; id -= LSB(id))
            sum += m_Tree[id];
        return sum;
    }
    // Range query
    T query(int lo, int hi) const { return query(hi) - query(lo - 1); }

    // Update by increment
    void update(int id, T val)
    {
        if (id <= 0)
            throw std::invalid_argument("id has to be positive: id > 0");
        if (!val)
            return;
        
        m_Array[id] += val;
        int N = m_Tree.size();
        for (; id < N; id += LSB(id))
            m_Tree[id] += val;
    }
    // Update by setting
    void set(int id, T val) { update(id, val - m_Array[id]); }

    void print() const
    {
        std::cout << "Array:\t";
        for (size_t i = 1; i < m_Array.size(); i++) {
            std::cout << m_Array[i] << " ";
        }   std::cout << "\n";
        std::cout << "Tree:\t";
        for (size_t i = 1; i < m_Tree.size(); i++) {
            std::cout << m_Tree[i] << " ";
        }   std::cout << "\n";   
    }
};

int main(int argc, char const *argv[])
{
    FenwickTree ft(16);
    ft.set(1, 1);
    ft.print();
    ft.set(2, 2);
    ft.print();
    ft.set(15, 2);
    ft.print();

    std::cout << "\nConstruct Fenwick tree from an existing array" << std::endl;
    FenwickTree ft2({1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0});
    ft2.print();

    std::cout << "\nConstruct Fenwick tree from an existing array" << std::endl;
    FenwickTree ft3({3, 4, -2, 7, 3, 11, 5, -8, -9, 2, 4, -8});
    ft3.print();
}
