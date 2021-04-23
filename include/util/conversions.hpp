#ifndef CONVERSIONS_HPP
#define CONVERSIONS_HPP

namespace conversions {

constexpr long double pi = 3.1415926535L;

constexpr long double operator"" _in(long double length) {
    return length;
}
constexpr long double operator"" _in(unsigned long long length) {
    return length;
}
constexpr long double operator"" _ft(long double length) {
    return length * 12;
}
constexpr long double operator"" _ft(unsigned long long length) {
    return length * 12;
}

constexpr long double operator"" _deg(long double angle) {
    return angle;
}
constexpr long double operator"" _deg(unsigned long long angle) {
    return angle;
}
constexpr long double operator"" _rad(long double angle) {
    return angle * 180 / pi;
}
constexpr long double operator"" _rad(unsigned long long angle) {
    return angle * 180 / pi;
}

constexpr long double radians(long double degrees) {
    return degrees * pi / 180;
}

constexpr long double degrees(long double radians) {
    return radians * 180 / pi;
}

} // namespace conversions

#endif