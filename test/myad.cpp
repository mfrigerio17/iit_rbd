#include <iostream>
//#include <type_traits>

///*
#include <iit/rbd/utils.h>
#include <iit/rbd/compact_transform.h>
#include <iit/robcogen/scalar/cppad.h>
#include <iit/robcogen/scalar/myad.h>
#include <iit/robcogen/jacs.h>
#include <fancy/rcg2/constants.h>
#include <fancy/rcg2/transforms.h>
//*/

//TODO: rewrite this test with some custom transform, removing dependencies on Fancy
int main()
{
    using namespace iit::rbd;
    using Scalar = maf::ADScalar;
    using B_XH_A = iit::rbd::B_XH_A<Scalar>;
    //using Scalar = iit::robcogen::cppad::Double;
    ///*
    fancy::rcg2::Transforms<Scalar> xt;
    fancy::rcg2::JointState<Scalar> q;
    q.setZero();
    q(0).x = M_PI/3;
    q(0).x = 0;
    q(1).x = 0.41;
    q(2).x = M_PI/7;
    q(1).dx = 1;
    xt.update(q);

    iit::rbd::TransformBase<Scalar> H{xt.m_link1_X_base0.ct};

    iit::rbd::Vec3<Scalar> p0 = H.BA_vect_in_B_coords();
    iit::rbd::Vec3<Scalar> z0 = H.A_rotmx_B().transpose().block<3,1>(0,2);

    H = xt.m_link2_X_link1.compose(H);
    //iit::rbd::Vec3<Scalar> p1 = H.BA_vect_in_B_coords();
    iit::rbd::Vec3<Scalar> z1 = H.A_rotmx_B().transpose().block<3,1>(0,2);

    H = xt.m_link3_X_link2.compose(H);
    iit::rbd::Vec3<Scalar> p2 = H.BA_vect_in_B_coords();
    iit::rbd::Vec3<Scalar> z2 = H.A_rotmx_B().transpose().block<3,1>(0,2);

    iit::rbd::Vec3<Scalar> p_ee = H.as<B_XH_A>() * iit::rbd::Vec3<Scalar>{1,0,0};

    iit::rbd::PlainMatrix<Scalar,6,3> J;
    iit::robcogen::geometricJacobianColumn_revolute(p_ee, p0, z0, J.col(0));
    iit::robcogen::geometricJacobianColumn_prismatic(z1, J.col(1));
    iit::robcogen::geometricJacobianColumn_revolute(p_ee, p2, z2, J.col(2));

    std::cout << J << std::endl  << std::endl;

    std::cout << p_ee(0).dx << " " << p_ee(1).dx << " " << p_ee(2).dx << std::endl;
    return 0;
}
