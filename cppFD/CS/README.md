<!-- omit in toc -->
# Special Data Structures and Algorithms implemented in C++ STL

Examples usage of some common Computer Science Data Structures and Algorithms in the `STL` are shown here.

## Table of Contents

- [Table of Contents](#table-of-contents)
- [Sorting](#sorting)

-------

## Sorting

Refs: [stackoverflow](https://stackoverflow.com/q/5038895/11397588), [cprogramming.com](https://www.cprogramming.com/tutorial/computersciencetheory/sortcomp.html)

- `std::sort` is most likely to use *QuickSort*, or at least a variation over *QuickSort* called *IntroSort*, which "degenerates" to *HeapSort* when the recursion goes too deep.
- `std::stable_sort` is most likely to use *MergeSort*, because of the stability requirement. However note that *MergeSort* requires extra space in order to be efficient.
- Coding API is pretty much the same

    ```cpp
    #include <algorithm>

    std::sort(v.begin(), v.end());
    std::stable_sort(v.begin(), v.end());
    ```
