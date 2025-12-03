#include <iostream>

#include <iit/rbd/scalar_traits.h>
#include <iit/robcogen/scalar/cppad.h>

// Test that linearPart() and angularPart() can deduce the template argument,
int main()
{
    using namespace std;
    using namespace iit;

    using Double = typename rbd::DoubleTraits::Scalar;
    using Cppad  = typename robcogen::CppADDoubleTraits::Scalar;

    rbd::Velocity<Double> v1 = {1,2,3,4,5,6};
    rbd::Velocity<Double> v2;

    rbd::linearPart(v2) = rbd::angularPart(v1);
    cout << v2 << endl;



    const rbd::Velocity<Cppad> v3 = {1,1,1,11,222,3333};
    rbd::Velocity<Cppad> v4;
    rbd::angularPart(v4) = rbd::linearPart(v3);
    cout << v4 << endl;

    return 0;
}
