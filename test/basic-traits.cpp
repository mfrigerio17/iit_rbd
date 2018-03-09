#include <iostream>
#include "basic-traits.h"

using namespace iit;
using namespace std;


int main()
{
    cout << "Float traits:" << endl;
    test< rbd::FloatTraits >();

    cout << endl;

    cout << "Double traits:" << endl;
    test< rbd::DoubleTraits >();

    cout << endl;
#ifdef IITRBD_CPPAD_IS_THERE
    cout << "CppAD Float traits:" << endl;
    test< robcogen::CppADFloatTraits >();

    cout << endl;

    cout << "CppAD Double traits:" << endl;
    test< robcogen::CppADDoubleTraits >();
#endif

    cout  << endl << "Test of the traits-selector" << endl;
    testTraitsSelector();
}
