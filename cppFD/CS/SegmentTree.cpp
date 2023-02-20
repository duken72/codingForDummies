#include <iostream>
#include <vector>

// https://cp-algorithms.com/data_structures/segment_tree.html
template <typename T = int>
class SegmentTree {
    static T merge(const T &a, const T &b) { return a + b; }
    int m_size;
    std::vector<T> m_array;

    void build(const int& id, const int& low, const int& high,
            const std::vector<T>& in_array)
    {
        if (low == high) {
            m_array[id] = in_array[low];
            return;
        }
        int mid = low + (high - low) / 2;
        build(id * 2 + 1, low, mid, in_array);
        build(id * 2 + 2, mid + 1, high, in_array);
        m_array[id] = merge(m_array[id * 2 + 1], m_array[id * 2 + 2]);
    }

    T query(const int& id, const int& low, const int& high,
            const int& tgt_low, const int& tgt_high) const
    {
        if (low == tgt_low && high == tgt_high)
            return m_array[id];
        int mid = low + (high - low) / 2;
        if (tgt_high <= mid)
            return query(id * 2 + 1, low, mid, tgt_low, tgt_high);
        if (tgt_low > mid)
            return query(id * 2 + 2, mid + 1, high, tgt_low, tgt_high);
        return merge(
            query(id * 2 + 1, low, mid, tgt_low, mid),
            query(id * 2 + 2, mid + 1, high, mid + 1, tgt_high)); 
    }

    void update(const int& id, const int& low, const int& high,
            const int& target, const T& val)
    {
        if (target < low || target > high)
            return;
        if (low == high) {
            m_array[id] = val;
            return;
        }
        int mid = low + (high - low) / 2;
        update(id * 2 + 1, low, mid, target, val);
        update(id * 2 + 2, mid + 1, high, target, val);
        m_array[id] = merge(m_array[id * 2 + 1], m_array[id * 2 + 2]);
    }

public:
    SegmentTree(const std::vector<T>& array)
    : m_size(array.size()), m_array(4 * m_size) {
        build(0, 0, m_size - 1, array);
    }

    T query(const int& low, const int& high) const {
        return query(0, 0, m_size - 1, low, high);
    }

    T query(const int& id) const {
        return query(0, 0, m_size - 1, id, id);
    }
    
    void update(const int& id, const T &val) {
        update(0, 0, m_size - 1, id, val);
    }
};

int main(int argc, char const *argv[])
{
    std::vector<int> v = { 1, 4, 4, 8, 3, 7 };
    SegmentTree<int> st(v);

    for (auto val : v) {
        std::cout << val << " ";
    }   std::cout << std::endl;
    
    std::cout << "Sum in range [0, 0]: " << st.query(0) << std::endl;
    std::cout << "Sum in range [0, 1]: " << st.query(0, 1) << std::endl;
    std::cout << "Sum in range [2, 5]: " << st.query(2, 5) << std::endl;

    std::cout << "Setting id=1 to value=9" << std::endl;
    st.update(1, 9);
    std::cout << "Sum in range [0, 1]: " << st.query(0, 1) << std::endl;

    std::cout << "Setting id=2 to value=7" << std::endl;
    st.update(2, 7);
    std::cout << "Sum in range [2, 5]: " << st.query(2, 5) << std::endl;

    return 0;
}
