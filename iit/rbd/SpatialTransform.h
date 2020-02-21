
#ifndef IIT_RBD_SPATIAL_TRANSFORM_H_
#define IIT_RBD_SPATIAL_TRANSFORM_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/StateDependentMatrix.h>
#include <iit/rbd/internals.h>


namespace iit {
namespace rbd {


template<class State, class ActualMatrix>
class SpatialTransform : public StateDependentBase<State, ActualMatrix>
{
private:
    typedef typename State::Scalar Scalar;

public:
    using StateDependentBase<State, ActualMatrix>::operator();

    template <typename Derived>
    void A_XM_B(const MatrixBase<Derived>& v_b, Vec6<Scalar>& v_a)
    {
        angularPart(v_a) = a_R_b * angularPart(v_b);
        linearPart(v_a)  = internal::cross(r_ab_A, angularPart(v_a)) + a_R_b *linearPart(v_b);
    }

    template <typename Derived>
    void B_XM_A(const MatrixBase<Derived>& v_a, Vec6<Scalar>& v_b)
    {
        angularPart(v_b) = a_R_b.transpose() * angularPart(v_a);
        linearPart(v_b)  = a_R_b.transpose() * (internal::cross(-r_ab_A, angularPart(v_a)) + linearPart(v_a));
    }

    template <typename Derived>
    void A_XF_B(const MatrixBase<Derived>& v_B, Vec6<Scalar>& v_A)
    {
        linearPart(v_A) = a_R_b * linearPart(v_B);
        angularPart(v_A)=  internal::cross(r_ab_A, linearPart(v_A)) + a_R_b * angularPart(v_B);
    }

    template <typename Derived>
    void B_XF_A(const MatrixBase<Derived>& v_A, Vec6<Scalar>& v_B)
    {
        linearPart(v_B) = a_R_b.transpose() * linearPart(v_A);
        angularPart(v_B)= a_R_b.transpose() * (internal::cross(-r_ab_A, linearPart(v_A)) + angularPart(v_A));
    }
protected:
    Mat33<Scalar> a_R_b;
    Vec3 <Scalar> r_ab_A;
};

}
}



#endif /* IIT_RBD_TRANSFORMSBASE_H_ */
