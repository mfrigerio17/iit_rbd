#include <iit/rbd/rbd.h>
#include <iit/robcogen/scalar/cppad.h>
#include <Eigen/Dense>


#include <iostream>

using namespace iit;
using namespace std;


using ScalarTraits    = iit::robcogen::CppADDoubleTraits;
using Scalar          = typename ScalarTraits::Scalar;
using PrimitiveScalar = typename ScalarTraits::ValueType;

int main()
{
    Eigen::Matrix<Scalar, 3,3> mx;
    mx.setRandom();
    cout << mx.inverse() << endl;

    Eigen::Matrix<Scalar, 1,3> row(1,1,1);
    Eigen::Matrix<Scalar, 3,1> col(2,2,2);
    Scalar scalar(0.5);
    cout << row * col + scalar << endl;
}

