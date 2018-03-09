/* CPYHDR { */
/*
 * This file is part of the 'iit-rbd' library.
 * Copyright © 2015 2016 2017, Marco Frigerio (marco.frigerio@iit.it)
 *
 * See the LICENSE file for more information.
 */
/* } CPYHDR */

#ifndef _IIT_RBD_INERTIAMATRIX_H_
#define _IIT_RBD_INERTIAMATRIX_H_

#include "rbd.h"


namespace iit {
namespace rbd {

/**
 * Dense 6x6 matrix that represents the 6D spatial inertia tensor.
 * See chapther 2 of Featherstone's "Rigid body dynamics algorithms".
 */
class InertiaMatrixDense : public Matrix66d {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    typedef Matrix66d Base;
    typedef MatrixBlock<const Base,3,3> Block33_const;
    typedef MatrixBlock<Base,3,3> Block33_t;

public:
    template<typename OtherDerived>
    InertiaMatrixDense& operator= (const MatrixBase<OtherDerived>& other);

    template<typename OtherDerived>
    InertiaMatrixDense& operator+= (const MatrixBase<OtherDerived>& other);

    InertiaMatrixDense();
    /**
     * See fill()
     */
    InertiaMatrixDense(double m, const Vector3d& com, const Matrix33d& I);

public:
    /**
     * Sets this 6x6 inertia tensor according to the given inertia properties.
     * All the values (ie the COM and the 3x3 tensor) must be expressed in the
     * same reference frame.
     *
     * No consistency checks are performed (Note: possibly changing in future).
     *
     * \param mass the total mass of the body
     * \param com the 3D vector with the position of the center of mass
     * \param tensor the classical 3x3 inertia tensor; this parameter should be
     *    expressed in the same coordinate frame as the center-of-mass vector.
     *    In other words, it is NOT treated as the inertia tensor with respect
     *    to a frame with origin in the center-of-mass, and the parallel axis
     *    theorem is NOT applied. The given tensor is copied as it is, in the
     *    appropriate 3x3 sub-block of this spatial tensor.
     */
    void fill(double m, const Vector3d& com, const Matrix33d& tensor);

    /** \name Components getters **/
    ///@{
    /**
     * @return the current value of the mass of the rigid body modeled by this
     * tensor
     */
    double getMass() const;
    /**
     * @return the position of the center-of-mass of the rigid body modeled by
     *   this spatial tensor
     */
    Vector3d getCOM() const;
    /**
     * @return the 3x3 block of this spatial tensor which corresponds to the
     *   sole rotational inertia, that is, the classical inertia tensor
     */
    const Block33_const get3x3Tensor() const;
    ///@}

    /** \name Modifiers **/
    ///@{
    /**
     * Scales the whole tensor according to the new value of the mass.
     *
     * The changes guarantee that the matrix remains positive definite (assuming
     * it was so before).
     *
     * This method does NOT change the center-of-mass property, while it does
     * change the moments of inertia. Intuitively, calling this method corresponds
     * to changing the mass-density of the body leaving its size and geometry
     * untouched.
     *
     * @param newMass the new value of the mass (always expressed in Kilograms);
     *    it MUST be positive, no checks are performed
     */
    void changeMass(double m);
    /**
     * Changes the position of the Center-Of-Mass of the rigid body modeled by
     * this tensor.
     *
     * In addition to the two off-diagonal 3x3 blocks, this method also modifies
     * the 3x3 block that corresponds to the classical inertia tensor, to keep
     * it consistent with the position of the center of mass. It does not change
     * the mass property. Cfr. chapter 2 of Featherstone's book on rigid body
     * dynamics algorithms.
     * TODO show some equations
     *
     * @param newcom a 3D vector specifying the position of the center of mass,
     *   expressed in meters
     */
    void changeCOM(const Vector3d& newcom);
    /**
     * Simply sets the 3x3 block that corresponds to the classical rotational
     * inertia
     * @param tensor the new 3x3 rotational inertia tensor
     */
    void changeRotationalInertia(const Matrix33d& tensor);
    ///@}
protected:
    void setTheFixedZeros();

    template<typename Vector>
    void setSkewSymmetricBlock(
            const MatrixBase<Vector>& v,
            Block33_t block);
private:
    void set(double m, const Vector3d& com, const Matrix33d& I);

};

class InertiaMatrixSparse : public SparseMatrixd {
    typedef SparseMatrixd Base;
};


#define block33 this->block<3,3>
#define data    (this->operator())


/* I do not care here about the creation of temporaries, thus I use const
 * refs to actual Eigen matrices, rather than the common base of the
 * matrix expression. If you haven't read Eigen docs, this comment is
 * completely obscure!
 */

inline InertiaMatrixDense::InertiaMatrixDense() : Base() {
    setTheFixedZeros();
}

/**
 * Initializes this 6x6 tensor according to the given inertia parameters.
 * \see fill()
 */
inline InertiaMatrixDense::InertiaMatrixDense(
        double mass, const Vector3d& cogPosition, const Matrix33d& tensor)
: Base()
{
    setTheFixedZeros();
    set(mass, cogPosition, tensor);
}

inline void InertiaMatrixDense::fill(double mass, const Vector3d& comPosition,
        const Matrix33d& tensor)
{
    set(mass, comPosition, tensor);
}

inline double InertiaMatrixDense::getMass() const
{
    return data(LX,LX);
}


inline Vector3d InertiaMatrixDense::getCOM() const
{
    return Vector3d(
            data(AZ,LY)/data(LX,LX), // X coordinate of the COM
            data(AX,LZ)/data(LX,LX), // Y
            data(AY,LX)/data(LX,LX));// Z
}

inline const InertiaMatrixDense::Block33_const InertiaMatrixDense::get3x3Tensor() const
{
    return block33(AX,AX);
}


inline void InertiaMatrixDense::changeMass(double newMass) {
    // Note the use of indices AX and hard-coded 0, to make it independent from
    //  the convention angular/linear
    this->block<3,6>(AX,0) *= newMass/getMass();
    block33(LX,AX) = block33(AX,LX).transpose();
    data(LX,LX) = data(LY,LY) = data(LZ,LZ) = newMass;
}

inline void InertiaMatrixDense::changeCOM(const Vector3d& newcom)
{
    // Update the angular-linear block according to the new COM position
    setSkewSymmetricBlock(  getMass() * newcom, block33(AX,LX));

    // Correct the 3x3 tensor. Use the block(AX,LX) which reflects the new COM
    //  and the block(LX,AX) which instead is still based on the previous value.
    //     I = I - m(cx)(cx)^T + m(c'x)(c'x)^T
    //  where cx is the skew symmetric matrix for the old COM vector, while
    //  c'x is the one for the new COM vector.
    block33(AX,AX) += (
            ( block33(AX,LX) * block33(AX,LX).transpose() ) -
            ( block33(LX,AX).transpose() * block33(LX,AX) )
                ) / getMass();
    // Update the linear-angular block
    block33(LX,AX) = block33(AX,LX).transpose();
    // alternatively:
    // setSkewSymmetricBlock( - getMass() * newcom, block33(LX,AX));
}

inline void InertiaMatrixDense::changeRotationalInertia(const Matrix33d& tensor)
{
    block33(AX,AX) = tensor;
}


inline void InertiaMatrixDense::set(
        double mass,
        const Vector3d& com,
        const Matrix33d& tensor)
{
    block33(AX,AX) = tensor;// + mass * comCrossMx * comCrossMx.transpose();

    data(AX,LY) = data(LY,AX) = - ( data(AY,LX) = data(LX,AY) = mass*com(Z) );
    data(AZ,LX) = data(LX,AZ) = - ( data(AX,LZ) = data(LZ,AX) = mass*com(Y) );
    data(AY,LZ) = data(LZ,AY) = - ( data(AZ,LY) = data(LY,AZ) = mass*com(X) );
    // Equivalent, just slightly less efficient (probably because of Eigen's blocks and repeated product):
    //setSkewSymmetricBlock(  mass * com, block33(AX,LX));
    //setSkewSymmetricBlock(- mass * com, block33(LX,AX));

    data(LX,LX) = data(LY,LY) = data(LZ,LZ) = mass;
}


template<typename OtherDerived>
inline InertiaMatrixDense& InertiaMatrixDense::operator=
        (const MatrixBase<OtherDerived>& other)
{
    // Here we silently assume that 'other' is also an inertia...
    //   Type safety would suggest to prevent the assignment of anything but
    // another inertia, but that would prevent, for example, the assignment
    // of matrix expressions. We also do not want to perform any check, for
    // performance reasons (remember this library is meant primarily to support
    // code generation, not to be a robust API for user applications).
    block33(AX,AX) = other.block<3,3>(AX,AX);
    data(AX,LY) = data(LY,AX) = - ( data(AY,LX) = data(LX,AY) = other(LX,AY) );
    data(AZ,LX) = data(LX,AZ) = - ( data(AX,LZ) = data(LZ,AX) = other(LZ,AX) );
    data(AY,LZ) = data(LZ,AY) = - ( data(AZ,LY) = data(LY,AZ) = other(LY,AZ) );
    data(LX,LX) = data(LY,LY) = data(LZ,LZ) = other(LX,LX);
    return *this;
}

template<typename OtherDerived>
inline InertiaMatrixDense& InertiaMatrixDense::operator+=
        (const MatrixBase<OtherDerived>& other)
{
    block33(AX,AX) += other.block<3,3>(AX,AX);
    data(AX,LY) = data(LY,AX) = - ( data(AY,LX) = (data(LX,AY) += other(LX,AY)) );
    data(AZ,LX) = data(LX,AZ) = - ( data(AX,LZ) = (data(LZ,AX) += other(LZ,AX)) );
    data(AY,LZ) = data(LZ,AY) = - ( data(AZ,LY) = (data(LY,AZ) += other(LY,AZ)) );
    data(LX,LX) = data(LY,LY) = (data(LZ,LZ) += other(LX,LX));
    return *this;
}


inline void InertiaMatrixDense::setTheFixedZeros()
{
    block33(LX,LX).setZero(); // the diagonal won't be zero, but who cares
    data(AX,LX) = data(AY,LY) = data(AZ,LZ) =
            data(LX,AX) = data(LY,AY) = data(LZ,AZ) = 0;
}

template<typename Vector>
inline void InertiaMatrixDense::setSkewSymmetricBlock(
        const MatrixBase<Vector>& v, Block33_t block)
{
    block(X,Y) = - ( block(Y,X) = v(Z) );
    block(Z,X) = - ( block(X,Z) = v(Y) );
    block(Y,Z) = - ( block(Z,Y) = v(X) );
}


#undef block33
#undef data

}
}


#endif /* INERTIAMATRIX_H_ */
