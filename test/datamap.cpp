#include <iostream>
#include <iit/rbd/data_map.h>

int main()
{
    enum Dummy { link1=0, link2, link3, link4, __N };

    using Map = iit::rbd::DataMap<int, Dummy::__N, Dummy>;

    Map m1{5};
    Map m2{m1};

    for(auto x : m1) {
        std::cout << x << " ";
    }
    std::cout << std::endl;
    for(auto x : m2) {
        std::cout << x << " ";
    }
    std::cout << std::endl;

    m2 = 7;
    m1 = m2;
    for(auto x : m1) {
        std::cout << x << " ";
    }
    std::cout << std::endl;
    for(auto x : m2) {
        std::cout << x << " ";
    }
    std::cout << std::endl;

    m1[link1] = 1;
    m1[link2] = 2;
    m1[link3] = 3;
    for(auto x : m1) {
        std::cout << x << " ";
    }
    std::cout << std::endl;
    return 0;
}
