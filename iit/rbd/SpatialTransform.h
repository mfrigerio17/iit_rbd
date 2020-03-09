
#ifndef IIT_RBD_SPATIAL_TRANSFORM_H_
#define IIT_RBD_SPATIAL_TRANSFORM_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/StateDependentMatrix.h>
#include <iit/rbd/internals.h>


namespace iit {
namespace rbd {


template<typename Scalar>
struct CTransformCore
{
    template <typename Derived>
    void A_XM_B(const MatrixBase<Derived>& v_b, Vec6<Scalar>& v_a) const
    {
        angularPart(v_a) = a_R_b * angularPart(v_b);
        linearPart(v_a)  = internal::cross(r_ab_A, angularPart(v_a)) + a_R_b *linearPart(v_b);
    }

    template <typename Derived>
    void B_XM_A(const MatrixBase<Derived>& v_a, Vec6<Scalar>& v_b) const
    {
        angularPart(v_b) = a_R_b.transpose() * angularPart(v_a);
        linearPart(v_b)  = a_R_b.transpose() * (internal::cross(-r_ab_A, angularPart(v_a)) + linearPart(v_a));
    }

    template <typename Derived>
    void A_XF_B(const MatrixBase<Derived>& v_B, Vec6<Scalar>& v_A) const
    {
        linearPart(v_A) = a_R_b * linearPart(v_B);
        angularPart(v_A)=  internal::cross(r_ab_A, linearPart(v_A)) + a_R_b * angularPart(v_B);
    }

    template <typename Derived>
    void B_XF_A(const MatrixBase<Derived>& v_A, Vec6<Scalar>& v_B) const
    {
        linearPart(v_B) = a_R_b.transpose() * linearPart(v_A);
        angularPart(v_B)= a_R_b.transpose() * (internal::cross(-r_ab_A, linearPart(v_A)) + angularPart(v_A));
    }

    Mat33<Scalar> a_R_b;
    Vec3 <Scalar> r_ab_A;
};


#define STRUCT_DEFINING_OPERATOR_STAR(KEY)                                  \
    template<typename Scalar>                                               \
    struct KEY : public CTransformCore<Scalar>                            \
    {                                                                       \
        template <typename Derived>                                         \
        rbd::Vec6<Scalar> operator*(const MatrixBase<Derived>& v_in) const  \
        {                                                                   \
            rbd::Vec6<Scalar> v2;                                           \
            CTransformCore<Scalar>::KEY(v_in, v2);                        \
            return v2;                                                      \
        }                                                                   \
    };

STRUCT_DEFINING_OPERATOR_STAR(A_XM_B)
STRUCT_DEFINING_OPERATOR_STAR(B_XM_A)
STRUCT_DEFINING_OPERATOR_STAR(A_XF_B)
STRUCT_DEFINING_OPERATOR_STAR(B_XF_A)


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



#endif /* IIT_RBD_TRANSFORMSBASE_H_ */
