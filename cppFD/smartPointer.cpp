// https://www.geeksforgeeks.org/smart-pointers-cpp/
// https://www.geeksforgeeks.org/auto_ptr-unique_ptr-shared_ptr-weak_ptr-2/

#include <memory>
#include <iostream>

class A
{
public:
    void show()
    {
        std::cout << "Run A::show()" << std::endl;
    }
};

int main()
{
    /* Deprecated since C++11
    std::auto_ptr<A> p1(new A);
    p1->show();
    std::cout << "Memory address of p1 " << p1.get() << std::endl;
    std::auto_ptr<A> p2(p1);
    p2->show();
    std::cout << "Memory address of p1 " << p1.get() << std::endl;
    std::cout << p2.get() << std::endl;
    */

    // UniquePtr
    std::cout << "unique_ptr example:" << std::endl;
    std::unique_ptr<A> p1(new A);
    p1->show();
    // returns the memory address of p1
    std::cout << "Memory address of p1 " << p1.get() << std::endl;

    // Error: can't copy unique_ptr
    // std::unique_ptr<A> p2 = p1;

    // transfers ownership to p2
    std::unique_ptr<A> p2 = move(p1);
    p2->show();
    std::cout << "Memory address of p1 " << p1.get() << std::endl;
    std::cout << "Memory address of p2 " << p2.get() << std::endl;
    std::cout << std::endl;

    // SmartPtr
    std::cout << "smart_ptr example:" << std::endl;
    std::shared_ptr<A> p3(new A);
    std::cout << "Memory address of p3 " << p3.get() << std::endl;
    p3->show();
    std::shared_ptr<A> p4(p3);
    p4->show();
    std::cout << "Memory address of p3 " << p3.get() << std::endl;
    std::cout << "Memory address of p4 " << p4.get() << std::endl;  
    // Returns the number of shared_ptr objects
    // referring to the same managed object.
    std::cout << "Counts: " << p3.use_count() << std::endl;
    std::cout << "Counts: " << p4.use_count() << std::endl;
    // Relinquishes ownership of p1 on the object
    // and pointer becomes NULL
    p3.reset();
    std::cout << "Memory address of p3 " << p3.get() << std::endl;
    std::cout << "Counts: " << p4.use_count() << std::endl;
    std::cout << "Memory address of p4 " << p4.get() << std::endl;
    std::cout << std::endl;

    // Good practice
    std::shared_ptr<A> p5 = std::make_shared<A>();
    std::unique_ptr<A> p5 = std::make_unique<A>();
}
