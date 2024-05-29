#pragma once

#include <array>

namespace beacon
{
using QuinticCoefficients = std::array<double, 6>;

class QuinticPolynomial
{
public:
  QuinticPolynomial();
  QuinticPolynomial(QuinticCoefficients coff)
    : coff_(coff)
  {}

  double f(double x) const;

  double df(double x) const;

  double ddf(double x) const;

  double dddf(double x) const;

  void SetCoff(QuinticCoefficients coff);

private:
  QuinticCoefficients coff_{};
};
} // namespace beacon