/* CPYHDR { */
/*
 * SPDX-FileCopyrightText: © 2026 Marco Frigerio
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of the 'iit-rbd' library.
 */
/* } CPYHDR */

#include <iostream>

#include <iit/rbd/scalar_traits.h>


// Test that linearPart() and angularPart() can deduce the template argument,
int main()
{
    using namespace std;
    using namespace iit;

    using Double = typename rbd::DoubleTraits::Scalar;
    using Float  = typename rbd::FloatTraits::Scalar;

    rbd::Velocity<Double> v1 = {1,2,3,4,5,6};
    rbd::Velocity<Double> v2;

    rbd::linearPart(v2) = rbd::angularPart(v1);
    cout << v2 << endl;



    const rbd::Velocity<Float> v3 = {1,1,1,11,222,3333};
    rbd::Velocity<Float> v4;
    rbd::angularPart(v4) = rbd::linearPart(v3);
    cout << v4 << endl;


    //rbd::linearPart(v2)  = rbd::linearPart(v3);   // ERR, cant mix matrices of different numeric types
    //rbd::angularPart(v4) = rbd::angularPart(v2);  // ERR

    return 0;
}
