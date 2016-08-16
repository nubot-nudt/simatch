/*
 * This file is modified from the sources of Gazebo 7.0
 *
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef _GAZEBO_MATH_QUATERNION_HH_
#define _GAZEBO_MATH_QUATERNION_HH_

#include <math.h>
#include <iostream>
#include <cmath>

#include "Helpers.hh"

namespace gazebo
{
  namespace math
  {
  /// \addtogroup gazebo_math
  /// \{

  /// \class Quaternion Quaternion.hh math/gzmath.hh
  /// \brief A quaternion class
  class Quaternion
  {
    /// \brief Default Constructor
    public: Quaternion();

    /// \brief Constructor
    /// \param[in] _w W param
    /// \param[in] _x X param
    /// \param[in] _y Y param
    /// \param[in] _z Z param
    public: Quaternion(const double &_w, const double &_x, const double &_y,
                        const double &_z);

    /// \brief Copy constructor
    /// \param[in] _qt Quaternion to copy
    public: Quaternion(const Quaternion &_qt);

    /// \brief Destructor
    public: ~Quaternion();

    /// \brief Equal operator
    /// \param[in] _qt Quaternion to copy
    public: Quaternion &operator =(const Quaternion &_qt);

    /// \brief Invert the quaternion
    public: void Invert();

    /// \brief Get the inverse of this quaternion
    /// \return Inverse quaternion
    public: inline Quaternion GetInverse() const
            {
              double s = 0;
              Quaternion q(this->w, this->x, this->y, this->z);

              // use s to test if quaternion is valid
              s = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

              if (math::equal(s, 0.0))
              {
                q.w = 1.0;
                q.x = 0.0;
                q.y = 0.0;
                q.z = 0.0;
              }
              else
              {
                // deal with non-normalized quaternion
                // div by s so q * qinv = identity
                q.w =  q.w / s;
                q.x = -q.x / s;
                q.y = -q.y / s;
                q.z = -q.z / s;
              }
              return q;
            }

    /// \brief Set the quaternion to the identity
    public: void SetToIdentity();

    /// \brief Return the logarithm
    /// \return the log
    public: Quaternion GetLog() const;

    /// \brief Return the exponent
    /// \return the exp
    public: Quaternion GetExp() const;

    /// \brief Normalize the quaternion
    public: void Normalize();

    /// \brief Set the quaternion from an axis and angle
    /// \param[in] _x X axis
    /// \param[in] _y Y axis
    /// \param[in] _z Z axis
    /// \param[in] _a Angle in radians
    public: void SetFromAxis(double _x, double _y, double _z, double _a);

    /// \brief Set this quaternion from 4 floating numbers
    /// \param[in] _u u
    /// \param[in] _x x
    /// \param[in] _y y
    /// \param[in] _z z
    public: void Set(double _u, double _x, double _y, double _z);

    /// \brief Set the quaternion from Euler angles.
    /// \param[in] _roll Roll angle (radians).
    /// \param[in] _pitch Pitch angle (radians).
    /// \param[in] _yaw Yaw angle (radians).
    public: void SetFromEuler(double _roll, double _pitch, double _yaw);

    /// \brief Get the Euler yaw angle in radians
    /// \return the yaw component
    public: double GetYaw();

    /// \brief Addition operator
    /// \param[in] _qt quaternion for addition
    /// \return this quaternion + _qt
    public: Quaternion operator+(const Quaternion &_qt) const;

    /// \brief Addition operator
    /// \param[in] _qt quaternion for addition
    /// \return this quaternion + qt
    public: Quaternion operator+=(const Quaternion &_qt);

    /// \brief Subtraction operator
    /// \param[in] _qt quaternion to subtract
    /// \return this quaternion - _qt
    public: Quaternion operator-(const Quaternion &_qt) const;

    /// \brief Subtraction operator
    /// \param[in] _qt Quaternion for subtraction
    /// \return This quaternion - qt
    public: Quaternion operator-=(const Quaternion &_qt);

    /// \brief Multiplication operator
    /// \param[in] _q Quaternion for multiplication
    /// \return This quaternion multiplied by the parameter
    public: inline Quaternion operator*(const Quaternion &_q) const
            {
              return Quaternion(
                  this->w*_q.w - this->x*_q.x - this->y*_q.y - this->z*_q.z,
                  this->w*_q.x + this->x*_q.w + this->y*_q.z - this->z*_q.y,
                  this->w*_q.y - this->x*_q.z + this->y*_q.w + this->z*_q.x,
                  this->w*_q.z + this->x*_q.y - this->y*_q.x + this->z*_q.w);
            }

    /// \brief Multiplication operator by a scalar.
    /// \param[in] _f factor
    /// \return quaternion multiplied by the scalar
    public: Quaternion operator*(const double &_f) const;

    /// \brief Multiplication operator
    /// \param[in] _qt Quaternion for multiplication
    /// \return This quaternion multiplied by the parameter
    public: Quaternion operator*=(const Quaternion &qt);

    /// \brief Equal to operator
    /// \param[in] _qt Quaternion for comparison
    /// \return True if equal
    public: bool operator ==(const Quaternion &_qt) const;

    /// \brief Not equal to operator
    /// \param[in] _qt Quaternion for comparison
    /// \return True if not equal
    public: bool operator!=(const Quaternion &_qt) const;

    /// \brief Unary minus operator
    /// \return negates each component of the quaternion
    public: Quaternion operator-() const;

    /// \brief See if a quaternion is finite (e.g., not nan)
    /// \return True if quaternion is finite
    public: bool IsFinite() const;

    /// \brief Correct any nan values in this quaternion
    public: inline void Correct()
            {
              if (!std::isfinite(this->x))
                this->x = 0;
              if (!std::isfinite(this->y))
                this->y = 0;
              if (!std::isfinite(this->z))
                this->z = 0;
              if (!std::isfinite(this->w))
                this->w = 1;

              if (math::equal(this->w, 0.0) &&
                  math::equal(this->x, 0.0) &&
                  math::equal(this->y, 0.0) &&
                  math::equal(this->z, 0.0))
              {
                this->w = 1;
              }
            }

    /// \brief Round all values to _precision decimal places
    /// \param[in] _precision the precision
    public: void Round(int _precision);

    /// \brief Dot product
    /// \param[in] _q the other quaternion
    /// \return the product
    public: double Dot(const Quaternion &_q) const;

    /// \brief Spherical quadratic interpolation
    /// given the ends and an interpolation parameter between 0 and 1
    /// \param[in] _ft the interpolation parameter
    /// \param[in] _rkP the beginning quaternion
    /// \param[in] _rkA first intermediate quaternion
    /// \param[in] _rkB second intermediate quaternion
    /// \param[in] _rkQ the end quaternion
    /// \param[in] _shortestPath when true, the rotation may be inverted to
    /// get to minimize rotation
    /// \return The result of the quadratic interpolation
    public: static Quaternion Squad(double _fT, const Quaternion &_rkP,
                const Quaternion &_rkA, const Quaternion &_rkB,
                const Quaternion &_rkQ, bool _shortestPath = false);

    /// \brief Spherical linear interpolation between 2 quaternions,
    /// given the ends and an interpolation parameter between 0 and 1
    /// \param[in] _ft the interpolation parameter
    /// \param[in] _rkP the beginning quaternion
    /// \param[in] _rkQ the end quaternion
    /// \param[in] _shortestPath when true, the rotation may be inverted to
    /// get to minimize rotation
    /// \return The result of the linear interpolation
    public: static Quaternion Slerp(double _fT, const Quaternion &_rkP,
                const Quaternion &_rkQ, bool _shortestPath = false);

    /// \brief w value of the quaternion
    public: double w;

    /// \brief x value of the quaternion
    public: double x;

    /// \brief y value of the quaternion
    public: double y;

    /// \brief z value of the quaternion
    public: double z;

  };

  }
}
#endif
