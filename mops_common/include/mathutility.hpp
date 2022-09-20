#pragma once

#include <boost/math/constants/constants.hpp>

#include <algorithm>
#include <cmath>
#include <type_traits>

/*!
 * Math utilities.
 */
namespace math {

// Import boost::math constants into namespace
using boost::math::double_constants::pi;
using boost::math::double_constants::third_pi;
using boost::math::double_constants::half_pi;
using boost::math::double_constants::two_pi;
using boost::math::double_constants::pi_sqr;

/**
 * \brief Linear value mapping (feature scaling) to the range [a,b].
 *
 * \param x The value to be mapped.
 * \param xmin The minimum value that \p x can take on.
 * \param xmax The maximum value that \p x can take on.
 * \param a The lower bound.
 * \param b The upper bound.
 * \return The mapped value.
 */
template<typename T>
inline constexpr T map_linear(const T& x, const T& xmin, const T& xmax, const T& a, const T& b)
{
    return a + (x - xmin) * (b - a) / (xmax - xmin);
}

/*!
 * \brief Check whether two numbers are close to each other.
 *
 * \param a The first number.
 * \param b The second number.
 * \param rel_tol The relative tolerance.
 * \param abs_tol The minimum absolute tolerance – useful for comparisons near zero.
 *
 * \return True of numbers are close and false otherwise.
 */
inline constexpr bool isclose(double a, double b, double rel_tol = 1.0e-9, double abs_tol = 0.0)
{
    return std::abs(a - b) <= std::max(rel_tol * std::max(std::abs(a), std::abs(b)), abs_tol);
}

/*!
 * \brief Convert angle \p x from radians to degrees.
 *
 * \param x an angle in radians.
 * \return The angle in degrees.
 */
inline constexpr float degrees(float x)
{
    return x * 180.0f / boost::math::constants::pi<float>();
}

/*!
 * \overload
 */
inline constexpr double degrees(double x)
{
    return x * 180.0 / boost::math::constants::pi<double>();
}

/*!
 * \overload
 */
template<typename T>
inline constexpr typename std::enable_if<std::is_integral<T>::value, double>::type
degrees(T x)
{
    return double(x) * 180.0 / boost::math::constants::pi<double>();
}

/*!
 * \brief Convert angle \p x from degrees to radians.
 *
 * \param x an angle in degrees.
 * \return The angle in radians.
 */
inline constexpr float radians(float x)
{
    return x * boost::math::constants::pi<float>() / 180.0f;
}

/*!
 * \overload
 */
inline constexpr double radians(double x)
{
    return x * boost::math::constants::pi<double>() / 180.0;
}

/*!
 * \overload
 */
template<typename T>
inline constexpr typename std::enable_if<std::is_integral<T>::value, double>::type
radians(T x)
{
    return double(x) * boost::math::constants::pi<double>() / 180.0;
}

/*!
 * \brief Normalizes angle \p x to the interval \f$[-\pi, \pi]\f$.
 *
 * \param x an angle in radians.
 * \return The normalized angle in radians.
 */
inline constexpr float normalize_angle(float x)
{
    // std::remainder returns the IEEE floating point remainder -
    // the exact value of x–(round(x/y)*y)
    return std::remainder(x, boost::math::constants::two_pi<float>());
}

/*!
 * \overload
 */
inline constexpr double normalize_angle(double x)
{
    return std::remainder(x, boost::math::constants::two_pi<double>());
}

/*!
 * \overload
 */
template<typename T>
inline constexpr typename std::enable_if<std::is_integral<T>::value, double>::type
normalize_angle(T x)
{
    return std::remainder(double(x), boost::math::constants::two_pi<double>());
}

/*!
 * \brief Normalizes angle \p x to the interval \f$[0, 2\pi]\f$.
 *
 * \param x an angle in radians.
 * \return The normalized angle in radians.
 */
inline constexpr float normalize_angle_positive(float x)
{
    constexpr auto pi_ = boost::math::constants::pi<float>();
    constexpr auto two_pi_ = boost::math::constants::two_pi<float>();
    return std::remainder(x - pi_, two_pi_) + pi_;
}

/*!
 * \overload
 */
inline constexpr double normalize_angle_positive(double x)
{
    constexpr auto pi_ = boost::math::constants::pi<double>();
    constexpr auto two_pi_ = boost::math::constants::two_pi<double>();
    return std::remainder(x - pi_, two_pi_) + pi_;
}

/*!
 * \overload
 */
template<typename T>
inline constexpr typename std::enable_if<std::is_integral<T>::value, double>::type
normalize_angle_positive(T x)
{
    constexpr auto pi_ = boost::math::constants::pi<double>();
    constexpr auto two_pi_ = boost::math::constants::two_pi<double>();
    return std::remainder(double(x) - pi_, two_pi_) + pi_;
}

} // namespace math
