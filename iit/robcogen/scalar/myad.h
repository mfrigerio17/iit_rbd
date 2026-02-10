/**
 * A trivial, didactical implementation of a custom scalar type implementing
 * Automatic Differentiation in forward mode.
 *
 * Includes the necessary type traits for iit::rbd and Eigen, in order to
 * actually use this type with RobCoGen-erated code
 */

#ifndef IIT_ROBCOGEN_MY_AD_SCALAR__H
#define IIT_ROBCOGEN_MY_AD_SCALAR__H

#include "../../rbd/scalar_traits.h"

namespace maf {

struct ADScalar
{
    constexpr ADScalar() {}
    constexpr ADScalar(double val) : x(val), dx(0.0) {}
    constexpr ADScalar(double val, double der) : x(val), dx(der) {}

    ADScalar& operator=(const ADScalar& rhs) {
        x  = rhs.x;
        dx = rhs.dx;
        return *this;
    }

    ADScalar& operator+=(const ADScalar& rhs)
    {
        x  += rhs.x;
        dx += rhs.dx;
        return *this;
    }

    double x {0.0};
    double dx{0.0};
};

inline std::ostream& operator<<(std::ostream& out, const ADScalar& rhs)
{
    out<<rhs.x;
    return out;
}

inline ADScalar operator*(const ADScalar& lhs, const ADScalar& rhs)
{
    return {lhs.x*rhs.x, lhs.dx*rhs.x + lhs.x*rhs.dx};
}

inline ADScalar operator*(const ADScalar& lhs, double rhs)
{
    return {lhs.x*rhs, lhs.dx*rhs};
}

inline ADScalar operator*(double lhs, const ADScalar& rhs)
{
    return {lhs*rhs.x, lhs*rhs.dx};
}

inline ADScalar operator+(const ADScalar& lhs, const ADScalar& rhs)
{
    return {lhs.x+rhs.x, lhs.dx + rhs.dx};
}

inline ADScalar operator-(const ADScalar& lhs, const ADScalar& rhs)
{
    return {lhs.x-rhs.x, lhs.dx - rhs.dx};
}

inline ADScalar operator-(const ADScalar& rhs)
{
    return {-rhs.x, -rhs.dx};
}

inline ADScalar sin (const ADScalar& v) { return {std::sin(v.x), std::cos(v.x)*v.dx};  }
inline ADScalar cos (const ADScalar& v) { return {std::cos(v.x),-std::sin(v.x)*v.dx};  }
inline ADScalar sqrt(const ADScalar& v) {
    const auto root{std::sqrt(v.x)};
    return {root, 1/(2*root) * v.dx};
}


}

namespace iit {
namespace rbd {

template<> struct BasicTraits<maf::ADScalar>
{
    using ValueType = double;
    using Scalar    = maf::ADScalar;
};

template<> struct ScalarTraits<maf::ADScalar> : public iit::rbd::ScalarTraitsCommons< BasicTraits<maf::ADScalar> >
{
    using ADScalar  = maf::ADScalar;
    using ValueType = double;
    using Scalar    = ADScalar;
    inline static ADScalar sin (const ADScalar& x) { return maf::sin(x);  }
    inline static ADScalar cos (const ADScalar& x) { return maf::cos(x);  }
    inline static ADScalar tan (const ADScalar& x) { return {std::tan(x.x),0.0};  }
    inline static ADScalar abs (const ADScalar& x) { return {std::abs(x.x),0.0};  }
    inline static ADScalar fabs(const ADScalar& x) { return {std::fabs(x.x),0.0}; }
    inline static ADScalar sqrt(const ADScalar& x) { return maf::sqrt(x); }
};


}
}

namespace Eigen
{
template<> struct NumTraits<maf::ADScalar> : NumTraits<double>
{};

template<typename BinOp>
struct ScalarBinaryOpTraits<maf::ADScalar, double, BinOp>{
    using ReturnType = maf::ADScalar;
};
template<typename BinOp>
struct ScalarBinaryOpTraits<double, maf::ADScalar, BinOp>
{
    using ReturnType = maf::ADScalar;
};

}

#endif
