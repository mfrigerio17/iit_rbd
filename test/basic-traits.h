/*
 * basic-traits.h
 *
 *  Created on: Jun 1, 2018
 *      Author: marco
 */

#ifndef IIT_RBD_TEST_BASIC_TRAITS_H_
#define IIT_RBD_TEST_BASIC_TRAITS_H_

#include <string>
#include <sstream>
#include <iostream>

#include "../iit/rbd/scalar_traits.h"
#include "../iit/robcogen/scalar/internals.h"

#ifdef IITRBD_CPPAD_IS_THERE
#include "../iit/robcogen/scalar/cppad.h"
#endif

template<typename ScalarTrait> inline std::string stringify()
{
    typedef typename ScalarTrait::ValueType value_t;
    typedef typename ScalarTrait::Scalar    scalar_t;

    std::ostringstream ss;
    ss << "ValueType: " << stringify<value_t>() << "\tScalar: " << stringify<scalar_t>();
    return ss.str();
}

template<> inline std::string stringify<double>()
{
    return std::string("double");
}
template<> inline std::string stringify<float>()
{
    return std::string("float");
}

#ifdef IITRBD_CPPAD_IS_THERE
template<> inline std::string stringify<iit::robcogen::cppad::Double>()
{
    return std::string("cppad::Double");
}
template<> inline std::string stringify<iit::robcogen::cppad::Float>()
{
    return std::string("cppad::Float");
}
#endif


template<typename Trait>
void test()
{
    std::cout << stringify<Trait>() << std::endl;

    typename Trait::template PlainMatrix<3,1> x, b;
    typename Trait::template PlainMatrix<3,3> A;
    A.setIdentity();
    b = typename Trait::template PlainMatrix<3,1>(1.0, 2.0, 3.0);
    x = Trait::solve(A, b);
    std::cout << x(0) << " " << x(1) << " " << x(2) << std::endl;

    typename Trait::Scalar foo = M_PI;
    std::cout << Trait::sin( foo ) << std::endl;
}

void testTraitsSelector()
{
    struct A {
        typedef typename iit::rbd::FloatTraits ScalarTraits;
    };

    struct B {};//does not have a field ScalarTraits, so the default one should be selected

    typedef typename iit::robcogen::internal::ScalarTraitsSelector< A >::trait ATrait;
    typedef typename iit::robcogen::internal::ScalarTraitsSelector< B >::trait BTrait;
    std::cout << stringify< ATrait >() << std::endl;
    std::cout << stringify< BTrait >() << std::endl;
}

#endif
