//
// Created by Aaron Smith on 7/29/17.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>
#include <math.h>

using namespace std;

class Trajectory {

private:
  vector<double> s_cooeficients;
  vector<double> d_cooeficients;

  /**
   * Calculates the derivative of a polynomial and returns the corresponding coefficients.
   * @param coefficients
   * @return
   */
  vector<double> differentiate(vector<double> coefficients) {
    vector<double> new_cos = {};
    for(int deg = 1; deg < coefficients.size(); deg++) {
      new_cos.push_back((deg + 1) * coefficients[deg]);
    }
    return new_cos;
  }

public:
  double t;

  Trajectory(vector<double> s_cooeficients, vector<double> d_cooeficients, double t) : s_cooeficients(s_cooeficients),
                                                                                       d_cooeficients(d_cooeficients),
                                                                                       t(t) { }


  double EvaluatePolynomial(vector<double> coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
      result += coeffs[i] * pow(x, i);
    }
    return result;
  }

  /**
   * Evaluates the polynomial function for s at time t.
   * @param t
   * @return s(t)
   */
  double S(double t) {
    return EvaluatePolynomial(s_cooeficients, t);
  }

  double SDot(double t) {
    return EvaluatePolynomial(GetSDotCoefficients(), t);
  }

  double SDotDot(double t) {
    return EvaluatePolynomial(GetSDotDotCoefficients(), t);
  }

  double SDotDotDot(double t) {
    return EvaluatePolynomial(GetSDotDotDotCoefficients(), t);
  }

  /**
   * Evaluates the polynomail function for d at time t.
   * @param t
   * @return d(t)
   */
  double D(double t) {
    return EvaluatePolynomial(d_cooeficients, t);
  }

  double DDot(double_t) {
    return EvaluatePolynomial(GetDDotCoefficients(), t);
  }

  double DDotDot(double t) {
    return EvaluatePolynomial(GetDDotDotCoefficients(), t);
  }

  /**
   * The methods below get the coefficients for s_dot, s_double_dot, d_dot, and d_double_dot respectively.
   */
  vector<double> GetSDotCoefficients() {
    return differentiate(s_cooeficients);
  }

  vector<double> GetSDotDotCoefficients() {
    return differentiate(GetSDotCoefficients());
  }

  vector<double> GetSDotDotDotCoefficients() {
    return differentiate(GetSDotDotCoefficients());
  }

  vector<double> GetDDotCoefficients() {
    return differentiate(d_cooeficients);
  }

  vector<double> GetDDotDotCoefficients() {
    return differentiate(GetDDotCoefficients());
  }

};

#endif //PATH_PLANNING_TRAJECTORY_H
