#ifndef IIT_ROBCOGEN_TPL_CPPAD_H
#define IIT_ROBCOGEN_TPL_CPPAD_H


#include <cppad/cppad.hpp>

#include "../../rbd/rbd.h"
#include "../../rbd/scalar_traits.h"


namespace iit {
namespace robcogen {

namespace cppad {


typedef CppAD::AD<double> Double;
typedef CppAD::AD<float>  Float;

template<typename Scalar>
struct TraitFuncs
{
    inline static Scalar sin (const Scalar& x) { return CppAD::sin(x); }
    inline static Scalar cos (const Scalar& x) { return CppAD::cos(x); }
    inline static Scalar sqrt(const Scalar& x) { return CppAD::sqrt(x); }
    inline static Scalar abs (const Scalar& x) { return CppAD::abs(x); }
};


} //namespace 'cppad'



template<typename CppADFloat>
struct BasicTraits
{
    typedef typename CppADFloat::value_type ValueType;
    typedef CppADFloat Scalar;
};


template<typename CppADFloat>
struct ScalarTraits : public rbd::ScalarTraitsCommons< BasicTraits<CppADFloat> >,
                      public cppad::TraitFuncs< typename BasicTraits<CppADFloat>::Scalar >
{};

typedef ScalarTraits< cppad::Double > CppADDoubleTraits;
typedef ScalarTraits< cppad::Float  > CppADFloatTraits;


} // namespace robcogen

namespace rbd {

template<> struct ScalarTraits<robcogen::cppad::Float> : public robcogen::CppADFloatTraits {};
template<> struct ScalarTraits<robcogen::cppad::Double>: public robcogen::CppADDoubleTraits {};

}

} // namespace iit




namespace Eigen {

template<> struct NumTraits<iit::robcogen::cppad::Float>  : NumTraits<float> {};
template<> struct NumTraits<iit::robcogen::cppad::Double> : NumTraits<double> {};

}

#endif

