/**
(C) Copyright 2011-2020 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Bruno Vilhena Adorno     (adorno@ieee.org)
- Murilo M. Marinho        (murilo@nml.t.u-tokyo.ac.jp)
- Mateus Rodrigues Martins (martinsrmateus@gmail.com)
*/

#ifndef DQ_H
#define DQ_H

#if defined(__GNUC__) || defined(__clang__)
#define DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED __declspec(deprecated)
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED
#endif

#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

#include <iostream>
using namespace Eigen;

namespace DQ_robotics{


class DQ{

    //Private methods
private:
    VectorXd q_() const;
    double q_(const int a) const;

    //Attributes
public:
    Matrix<double,8,1> q;

    //Static Methods
public:
    static DQ unitDQ( const double& rot_angle, const int& x_axis, const int& y_axis, const int& z_axis, const double& x_trans, const double& y_trans, const double& z_trans);
    //To comply with MATLAB
    const static DQ i;
    const static DQ j;
    const static DQ k;
    const static DQ E;

    //Methods
public:
    explicit DQ(const VectorXd& v);

    explicit DQ(const double& q0=0.0, const double& q1=0.0, const double& q2=0.0, const double& q3=0.0, const double& q4=0.0, const double& q5=0.0, const double& q6=0.0, const double& q7=0.0);

    //Methods
    DQ P() const;

    DQ D() const;

    DQ Re() const;

    DQ Im() const;

    DQ conj() const;

    DQ norm() const;

    DQ inv() const;

    DQ translation() const;

    DQ rotation() const;

    DQ rotation_axis() const;

    double rotation_angle() const;

    DQ log() const;

    DQ exp() const;

    DQ pow(const double a) const;

    DQ tplus() const;
    inline DQ T() const{return tplus();}

    DQ pinv() const;

    Matrix4d hamiplus4() const;

    Matrix4d haminus4() const;

    Matrix<double,8,8> hamiplus8() const;

    Matrix<double,8,8> haminus8() const;

    Vector3d vec3() const;

    Vector4d vec4() const;

    Matrix<double,6,1> vec6() const;

    Matrix<double,8,1> vec8() const;

    Matrix<double,8,8> generalized_jacobian() const;

    DQ normalize() const;

    DQ sharp() const;

    DQ Ad(const DQ& dq2) const;

    DQ Adsharp(const DQ& dq2) const;

    //Operator (= Overload)
    template <typename Scalar, typename scalar = std::enable_if<std::is_arithmetic<Scalar>::value>>
    DQ& operator=(const Scalar& t)
    {
        q = VectorXd::Zero(8);
        q(0)=t;
        return *this;
    };
    //Operator (-) Overload
    DQ operator-() const;
    //Operator (==) Overload
    bool operator==(const DQ& dq2) const;
    //Operator (!=) Overload
    bool operator!=(const DQ& dq2) const;

    //Conversion of DQ to other types
    explicit operator double() const;
    explicit operator int()    const;

    std::string to_string() const;
};//DQ Class END

//Operators
DQ P(const DQ& dq);

DQ D(const DQ& dq);

DQ Re(const DQ& dq);

DQ Im(const DQ& dq);

DQ conj(const DQ& dq);

DQ norm(const DQ& dq);

DQ inv(const DQ& dq);

DQ translation(const DQ& dq);

DQ rotation(const DQ& dq);

DQ rotation_axis(const DQ& dq);

double rotation_angle(const DQ& dq);

DQ log(const DQ& dq);

DQ exp(const DQ& dq);

DQ pow(const DQ& dq, const double& a);

DQ tplus(const DQ& dq);
inline DQ T(const DQ& dq){return tplus(dq);}

DQ pinv(const DQ& dq);

DQ dec_mult(const DQ& dq1, const DQ& dq2);

Matrix4d hamiplus4(const DQ& dq);

Matrix4d haminus4(const DQ& dq);

Matrix<double,8,8> hamiplus8(const DQ& dq);

Matrix<double,8,8> haminus8(const DQ& dq);

Matrix<double,8,8> generalized_jacobian(const DQ& dq);

Vector3d vec3(const DQ& dq);

Vector4d vec4(const DQ& dq);

Matrix<double,6,1> vec6(const DQ& dq);

Matrix<double,8,1> vec8(const DQ& dq);

Matrix4d crossmatrix4(const DQ& dq);

DQ normalize (const DQ& dq);

DQ sharp(const DQ& dq);

DQ cross(const DQ& dq1, const DQ& dq2);

DQ dot(const DQ& dq1, const DQ& dq2);

DQ Ad(const DQ& dq1, const DQ& dq2);

DQ Adsharp(const DQ& dq1, const DQ& dq2);

bool is_unit(const DQ& dq);

bool is_pure(const DQ& dq);

bool is_real(const DQ& dq);

bool is_real_number(const DQ& dq);

bool is_quaternion(const DQ& dq);

bool is_pure_quaternion(const DQ& dq);

bool is_line(const DQ& dq);

bool is_plane(const DQ& dq);

const double DQ_threshold = 1e-12;

/*************************************************************************
 ************** DUAL QUATERNION CONSTANTS AND OPERATORS ******************
 ************************************************************************/

//Operator (+) Overload
DQ operator+(const DQ& dq1, const DQ& dq2);

template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
inline DQ operator+(const DQ& dq, const Scalar& s) noexcept
{
    DQ ret(dq);
    ret.q(0)+=s;
    return ret;
}

template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
inline DQ operator+(const Scalar& s, const DQ& dq) noexcept
{
    return operator+(dq,s);
}

//Operator (-) Overload
//Binary
DQ operator-(const DQ& dq1, const DQ& dq2);

template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
inline DQ operator-(const Scalar& s, const DQ& dq) noexcept
{
    return operator+(dq,-s);
}

template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
inline DQ operator-(const DQ& dq, const Scalar& s) noexcept
{
    return operator+(dq,-s);
}

//Operator (*) Overload
DQ operator*(const DQ& dq1, const DQ& dq2);

template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
inline DQ operator*(const DQ& dq, const Scalar& s) noexcept
{
    return DQ(s*dq.q);
}

template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
inline DQ operator*(const Scalar& s, const DQ& dq) noexcept
{
    return operator*(dq,s);
}


//Operator (==) Overload
template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
inline bool operator==(const DQ& dq, const Scalar& s)
{
    return dq==DQ(s);
}

template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
inline bool operator==(const Scalar& s, const DQ& dq) noexcept
{
    return operator==(dq,s);
}


//Operator (!=) Overload
template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
inline bool operator!=(const DQ& dq, const Scalar& s)
{
    return !operator==(dq,s);
}

template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
inline bool operator!=(const Scalar& s, const DQ& dq) noexcept
{
    return !operator==(dq,s);
}

//Operator (<<) Overload
std::ostream& operator<<(std::ostream &os, const DQ& dq);

//Constants
Matrix<double,8,8> C8();
Matrix<double,4,4> C4();

const DQ E_ = DQ(0,0,0,0,1,0,0,0);
const DQ i_ = DQ(0,1,0,0,0,0,0,0);
const DQ j_ = DQ(0,0,1,0,0,0,0,0);
const DQ k_ = DQ(0,0,0,1,0,0,0,0);

}//Namespace DQRobotics

#endif // DQ_H
