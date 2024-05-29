#include "rviz/quintic_polynomial.h"
#include <cmath>


namespace beacon
{
QuinticPolynomial::QuinticPolynomial()
{
  coff_[0] = 1;
  coff_[1] = 1;
  coff_[2] = 2;
  coff_[3] = 3;
  coff_[4] = 4;
  coff_[5] = 5;
}

double QuinticPolynomial::f(double x) const
{
  return coff_[0] + coff_[1] * x + coff_[2] * std::pow(x, 2) +
         coff_[3] * std::pow(x, 3) + coff_[4] * std::pow(x, 4) +
         coff_[5] * std::pow(x, 5);
}


double QuinticPolynomial::df(double x) const
{
  return coff_[1] + 2 * coff_[2] * x + 3 * coff_[3] * std::pow(x, 2) +
         4 * coff_[4] * std::pow(x, 3) + 5 * coff_[5] * std::pow(x, 4);
}


double QuinticPolynomial::ddf(double x) const
{
  return 2 * coff_[2] + 6 * coff_[3] * x + 12 * coff_[4] * std::pow(x, 2) +
         20 * coff_[5] * std::pow(x, 3);
}


double QuinticPolynomial::dddf(double x) const
{
  return 6 * coff_[3] + 24 * coff_[4] * x + 60 * coff_[5] * std::pow(x, 2);
}


void QuinticPolynomial::SetCoff(QuinticCoefficients coff)
{
  coff_ = coff;
}

} // namespace beacon