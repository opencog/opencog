/*
 * opencog/spatial/math/Quaternion.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Samir Araujo
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <opencog/spatial/math/Quaternion.h>

using namespace opencog;
using namespace opencog::spatial::math;


Quaternion::Quaternion( void ) : x(0), y(0), z(0), w(1)
{
}

Quaternion::Quaternion(const Quaternion& q) : x(q.x), y(q.y), z(q.z), w(q.w)
{
}

Quaternion::Quaternion(double x, double y, double z, double w) : x(x), y(y), z(z), w(w)
{
}

Quaternion::Quaternion(const Vector3& axis, double angle)
{
    set(axis, angle);
}

double Quaternion::length(void) const
{
    return (double) std::sqrt( dot(*this) );
}

Quaternion& Quaternion::set(const Vector3& axis, double angle)
{
    double halfAngle = 0.5 * angle;
    double s = (double) std::sin(halfAngle);
    w = (double) std::cos(halfAngle);
    x = axis.x * s;
    y = axis.y * s;
    z = axis.z * s;
    return *this;
}

Quaternion& Quaternion::operator*=( const Quaternion & q)
{
    double nw = w * q.w - x * q.x - y * q.y - z * q.z;
    double nx = w * q.x + x * q.w + y * q.z - z * q.y;
    double ny = w * q.y + y * q.w + z * q.x - x * q.z;
    z = w * q.z + z * q.w + x * q.y - y * q.x;
    w = nw;
    x = nx;
    y = ny;
    return *this;
}

Quaternion& Quaternion::operator*=(double scale)
{
    w *= scale;
    x *= scale;
    y *= scale;
    z *= scale;
    return *this;
}

Quaternion& Quaternion::operator/=(double scale)
{
    w /= scale;
    x /= scale;
    y /= scale;
    z /= scale;
    return *this;
}

double Quaternion::dot(const Quaternion& q) const
{
    return x * q.x + y * q.y + z * q.z + w * q.w;
}

Quaternion& Quaternion::interpolateThis(const Quaternion& q, double t)
{
    if (*this != q) {
        double d = dot(q);
        double qx, qy, qz, qw;

        if (d < 0.0f) {
            qx = -q.x;
            qy = -q.y;
            qz = -q.z;
            qw = -q.w;
            d = -d;
        } else {
            qx = q.x;
            qy = q.y;
            qz = q.z;
            qw = q.w;
        } // else

        double f0, f1;

        if ((1 - d) > 0.1f) {
            double angle = (double) std::acos(d);
            double s = (double) std::sin(angle);
            double tAngle = t * angle;
            f0 = (double) std::sin(angle - tAngle) / s;
            f1 = (double) std::sin(tAngle) / s;
        } else {
            f0 = 1 - t;
            f1 = t;
        } // else

        x = f0 * x + f1 * qx;
        y = f0 * y + f1 * qy;
        z = f0 * z + f1 * qz;
        w = f0 * w + f1 * qw;
    } // if

    return *this;
}

Quaternion& Quaternion::normalize(void)
{
    return (*this /= length());
}

Quaternion Quaternion::interpolate(const Quaternion& q, double t)
{
    return Quaternion(*this).interpolateThis(q, t);
}

Vector3 Quaternion::rotate( const Vector3& v ) const
{
    // nVidia SDK implementation
    Vector3 uv, uuv;
    Vector3 qvec( x, y, z);
    uv = qvec.crossProduct(v);
    uuv = qvec.crossProduct(uv);
    uv *= (2.0f * w);
    uuv *= ( 2.0f );

    return v + uv + uuv;
}

Matrix3 Quaternion::getRotationMatrix( void )
{

    double fTx  = 2.0 * x;

    double fTy  = 2.0 * y;
    double fTz  = 2.0 * z;
    double fTwx = fTx * w;
    double fTwy = fTy * w;
    double fTwz = fTz * w;
    double fTxx = fTx * x;
    double fTxy = fTy * x;
    double fTxz = fTz * x;
    double fTyy = fTy * y;
    double fTyz = fTz * y;
    double fTzz = fTz * z;

    return Matrix3(
               1.0 -(fTyy + fTzz), fTxy - fTwz, fTxz + fTwy,
               fTxy + fTwz, 1.0 - (fTxx + fTzz), fTyz - fTwx,
               fTxz - fTwy, fTyz + fTwx, 1.0 - (fTxx + fTyy)
           );
}

Quaternion& Quaternion::operator+=( const Quaternion & q )
{
    w += q.w;
    x += q.x;
    y += q.y;
    z += q.z;
    return *this;
}

Quaternion Quaternion::operator+( const Quaternion& q )
{
    return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
}

Quaternion Quaternion::operator-( void )
{
    return Quaternion( -x, -y, -z, -w );
}

double Quaternion::getRoll( void ) const
{
    return getRoll( true );
}

double Quaternion::getRoll( bool reprojectAxis) const
{
    if (reprojectAxis) {
        // roll = atan2(localx.y, localx.x)
        // pick parts of xAxis() implementation that we need
        //double fTx  = 2.0*x;
        double fTy  = 2.0 * y;
        double fTz  = 2.0 * z;
        double fTwz = fTz * w;
        double fTxy = fTy * x;
        double fTyy = fTy * y;
        double fTzz = fTz * z;

        // Vector3(1.0-(fTyy+fTzz), fTxy+fTwz, fTxz-fTwy);

        return std::atan2(fTxy + fTwz, 1.0 - (fTyy + fTzz));

    } else {
        return std::atan2(2*(x*y + w*z), w*w + x*x - y*y - z*z);
    } // else
}

double Quaternion::getPitch(void) const
{
    return getPitch(true);
}

double Quaternion::getPitch(bool reprojectAxis) const
{
    if (reprojectAxis) {
        // pitch = atan2(localy.z, localy.y)
        // pick parts of yAxis() implementation that we need
        double fTx  = 2.0 * x;
        //double fTy  = 2.0*y;
        double fTz  = 2.0 * z;
        double fTwx = fTx * w;
        double fTxx = fTx * x;
        double fTyz = fTz * y;
        double fTzz = fTz * z;

        // Vector3(fTxy-fTwz, 1.0-(fTxx+fTzz), fTyz+fTwx);
        return std::atan2(fTyz + fTwx, 1.0 - (fTxx + fTzz));
    } else {
        // internal version
        return std::atan2(2*(y*z + w*x), w*w - x*x - y*y + z*z);
    }
}

double Quaternion::getYaw(void) const
{
    return getYaw(true);
}

double Quaternion::getYaw(bool reprojectAxis) const
{
    if (reprojectAxis) {
        // yaw = atan2(localz.x, localz.z)
        // pick parts of zAxis() implementation that we need
        double fTx  = 2.0 * x;
        double fTy  = 2.0 * y;
        double fTz  = 2.0 * z;
        double fTwy = fTy * w;
        double fTxx = fTx * x;
        double fTxz = fTz * x;
        double fTyy = fTy * y;

        return std::atan2(fTxz + fTwy, 1.0 - (fTxx + fTyy));
    } else {
        // internal version
        return std::asin(-2*(x*z - w*y));
    }
}

bool Quaternion::operator==( const Quaternion& q ) const
{
    return ( x == q.x && y == q.y && z == q.z && w == q.w );
}

bool Quaternion::operator!=( const Quaternion& q ) const
{
    return x != q.x || y != q.y || z != q.z || w != q.w;
}
