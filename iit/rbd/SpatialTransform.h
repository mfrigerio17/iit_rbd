#ifndef IIT_RBD_COMPACT_TRANSFORM_H_
#define IIT_RBD_COMPACT_TRANSFORM_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/StateDependentMatrix.h>
#include <iit/rbd/internals.h>
//#include <iostream>

namespace iit {
namespace rbd {


// // Just a dummy type to investigate what happens with temporary vectors with
// // the code below. Using this type should confirm that no useless copies of
// // vectors are performed, even without optimizations (g++ 5.4 (onwards?)).
// // See instead what happens when deliberately compiling with -fno-elide-constructors
//template<typename Scalar>
//struct Vec6Probe : public Vec6<Scalar>
//{
//    Vec6Probe() {
//        std::cout << "CTor" << std::endl;
//    }
//    Vec6Probe(const Vec6Probe<Scalar>& rhs) {
//        std::cout << "Copy CTor" << std::endl;
//    }
//};

/** A compact type that can act as different representations of a coordinate transform.
 *
 * An instance contains the minimum amount of data to encode a coordinate
 * transform for homogeneous coordinates and spatial vectors (both motion and
 * force): a rotation matrix and a translation vector.
 *
 * Different member functions implement the actual coordinate transformation for
 * the different representations.
 */
template<typename Scalar>
struct CTransformCore
{
    typedef Vec6<Scalar> vector6;
    //typedef Vec6Probe<Scalar> vector6; // use this to investigate about temporaries

    template <typename Derived>
    vector6 A_XM_B(const MatrixBase<Derived>& v_b) const
    {
        vector6 v_a;
        angularPart(v_a) = a_R_b * angularPart(v_b);
        linearPart(v_a)  = internal::cross(r_ab_A, angularPart(v_a)) + a_R_b *linearPart(v_b);
        return v_a;
    }

    template <typename Derived>
    vector6  B_XM_A(const MatrixBase<Derived>& v_a) const
    {
        vector6 v_b;
        angularPart(v_b) = a_R_b.transpose() * angularPart(v_a);
        linearPart(v_b)  = a_R_b.transpose() * (internal::cross(-r_ab_A, angularPart(v_a)) + linearPart(v_a));
        return v_b;
    }

    template <typename Derived>
    vector6  A_XF_B(const MatrixBase<Derived>& v_B) const
    {
        vector6 v_A;
        linearPart(v_A) = a_R_b * linearPart(v_B);
        angularPart(v_A)=  internal::cross(r_ab_A, linearPart(v_A)) + a_R_b * angularPart(v_B);
        return v_A;
    }

    template <typename Derived>
    vector6  B_XF_A(const MatrixBase<Derived>& v_A) const
    {
        vector6 v_B;
        linearPart(v_B) = a_R_b.transpose() * linearPart(v_A);
        angularPart(v_B)= a_R_b.transpose() * (internal::cross(-r_ab_A, linearPart(v_A)) + angularPart(v_A));
        return v_B;
    }

    template <typename Derived>
    Vec3<Scalar> A_XH_B(const MatrixBase<Derived>& v_b) const
    {
        return a_R_b * v_b + r_ab_A;
    }

    template <typename Derived>
    Vec3<Scalar> B_XH_A(const MatrixBase<Derived>& v_a) const
    {
        return a_R_b.transpose() * (v_a - r_ab_A);
    }

    Mat33<Scalar> a_R_b;
    Vec3 <Scalar> r_ab_A;
};


// Define a type for each of the product functions defined above; this type
// can then overload the product operator and implement it with its
// corresponding function.
// For example, for the type 'A_XM_B', we have that:
//
//  A_XM_B::operator*()  <---->   CTransformCore::A_XM_B()
//
// This will enable us to make a TransformBase type (see below) which can use
// the nicer looking operator*.
//
// We use a parametrized macro, and then instantiate it a number of times:

#define STRUCT_DEFINING_OPERATOR_STAR(KEY)                         \
    template<typename Scalar>                                               \
    struct KEY : public CTransformCore<Scalar>                              \
    {                                                                       \
        template <typename Derived>                                         \
        auto operator*(const MatrixBase<Derived>& v_in) const -> decltype(CTransformCore<Scalar>::KEY(v_in)) \
        {                                                                   \
            return CTransformCore<Scalar>::KEY(v_in);                   \
        }                                                                   \
    };

// The 'decltype' above make sure to use the same return type as the actual
// product function that we call, in CTransformCore. This ensures consistency
// if we change the typedef(s) inside CTransformCore.


STRUCT_DEFINING_OPERATOR_STAR(A_XM_B)
STRUCT_DEFINING_OPERATOR_STAR(B_XM_A)
STRUCT_DEFINING_OPERATOR_STAR(A_XF_B)
STRUCT_DEFINING_OPERATOR_STAR(B_XF_A)
STRUCT_DEFINING_OPERATOR_STAR(A_XH_B)
STRUCT_DEFINING_OPERATOR_STAR(B_XH_A)


template<typename STATE, typename Actual>
struct TransformBase : public StateDependentBase<STATE, Actual>
{
    CTransformCore<typename STATE::Scalar> ct;

    template<typename REPR>
    inline const REPR& as() const {
        return static_cast<const REPR&>( ct );
    }
};

}
}



#endif
