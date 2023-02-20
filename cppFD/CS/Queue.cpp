// Reference
// https://en.cppreference.com/w/cpp/container/queue

#include <iostream>
#include <queue>
 
int main()
{
    // Create a queue containing integers
    std::queue<int> q;
    q.push(7);
    q.push(5);
    q.push(16);
    q.push(8);
    q.push(13);
    q.pop();
 
    // Iterate and print values of deque
    for (; !q.empty(); q.pop())
        std::cout << q.front() << " ";
    
    std::cout << "\n";
}
