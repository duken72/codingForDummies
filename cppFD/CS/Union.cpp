#pragma GCC optimize("O2,unroll-loops")
#pragma GCC target("avx2,bmi,bmi2,lzcnt,popcnt")

#include <iostream>
#include <vector>
#include <exception>

class UnionFind
{
public:
    UnionFind(int size_in);
    ~UnionFind() {};
    int find(int id);
    bool connected(int id1, int id2) { return find(id1) == find(id2); }
    int getSetSize(int id) { return size_set[find(id)]; };
    int size() const { return m_size; };
    int numSet() const { return m_numSets; };
    void unify(int id1, int id2);
    void print() const;
private:
    int m_size, m_numSets;
    std::vector<int> id_set;
    std::vector<int> size_set;
};

UnionFind::UnionFind(int size_in)
{
    if (size_in <= 0)
        throw std::invalid_argument("Negative size.");
    m_size = size_in;
    m_numSets = size_in;
    id_set = std::vector<int>(m_size);
    size_set = std::vector<int>(m_size, 1);
    for (int i = 0; i < m_size; i++) {
        id_set[i] = i;
    }
}

int UnionFind::find(int id)
{
    if (id >= m_size)
        throw std::out_of_range("Element id is out of range.");
    int root = id;
    while (root != id_set[root]) {
        root = id_set[root];
    }
    // Path compression
    int next = id;
    while (id != root) {
        next = id_set[id];
        id_set[id] = root;
        id = next;
    }
    return root;
}

void UnionFind::unify(int id1, int id2)
{
    int r1 = find(id1);
    int r2 = find(id2);
    // return if already in the same set
    if (r1 == r2)
        return;
    // Merge smaller set to the larger one
    if (size_set[r1] < size_set[r2]) {
        size_set[r2] += size_set[r1];
        id_set[r1] = r2;
    }   else {
        size_set[r1] += size_set[r2];
        id_set[r2] = r1;
    }
    m_numSets--;
}

void UnionFind::print() const
{
    for (int id : id_set) {
        std::cout << id << " ";
    }   std::cout << "\n";
    for (int s : size_set) {
        std::cout << s << " ";
    }   std::cout << "\n";
}

int main(int argc, char const *argv[])
{
    // UnionFind uf(-7);
    UnionFind uf(7);
    uf.unify(1, 2);
    uf.unify(1, 3);
    uf.print();
    std::cout << uf.numSet() << std::endl;
    // std::cout << uf.find(9) << std::endl;
    return 0;
}
