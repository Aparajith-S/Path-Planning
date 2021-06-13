/// \file polyfunction.h
/// \brief polynomial class that computes n-order differential calculus for a given polynomial
/// \author s.aparajith@live.com
/// \date 31.05.2021
/// \copyright None reserved. MIT license
#ifndef POLY_FUNCTION_H
#define POLY_FUNCTION_H
#include"types.h"
#include"interface.h"
#include<vector>
namespace path_planner {
using interfaces::coefficients;

///  @class polyfunction class.
class function
{
public:
    explicit function() :
        m_time(-1.0),
        m_coeffs(coefficients())
    {}
    virtual ~function() {}
    function(const function&) = default;
    function& operator=(const function&) = default;
    
    /// @brief compute value of a polynomial at a given time
    /// @returns function(t) 
    double operator()(double time)
    {
        this->m_time = time;
        double power_term = 1.0;
        bool once = false;
        double result = 0.0;
        for (const auto coeff : m_coeffs)
        {
            result += coeff * power_term;
            if (!once)
            {
                power_term = time;
                once = true;
            }
            else
            {
                power_term *= power_term;
            }
        }
        return result;
    }

    /// @brief constructs the polynomial equation
    void constructEquation(const coefficients& coeffs)
    {
        this->m_coeffs = coeffs;
    }

    /// @brief differentiate a polynomial equation
    /// @details if a coeff is not passed then the function will use its own coefficients to differentiate
    /// @return function that is differentiated
    function differentiate(const coefficients& coeffs = coefficients())
    {
        function func;
        coefficients newCoeff = coefficients();
        const coefficients* oldCoeff;
        if (coeffs.size() == 0)
        {
            oldCoeff = &(this->m_coeffs);
        }
        else
        {
            oldCoeff = &coeffs;
        }
        double deg_mul = 1;
        if (oldCoeff->size())
        {
            for (auto iter = oldCoeff->begin() + 1;
                 iter != oldCoeff->end();
                 iter++)
            {
                newCoeff.push_back((*iter) * deg_mul);
                ++deg_mul;
            }
        }
        func.constructEquation(newCoeff);
        return func;
    }

    /// @brief returns the function and its N derivatives. default returns vector of {f, fdot,fdotdot}
    /// @details if a coeff is not passed then the function will use its own coefficients to differentiate
    /// @return vector of N differentiated functions. default order 3 to 1.
    std::vector<function> differentiateNtimes(const coefficients& i_coeffs = coefficients(), const std::uint8_t i_order = 3)
    {
        function func;
        std::vector<function> returnValue;
        if (i_coeffs.size() == 0)
        {
            func = *this;
        }
        else
        {
            func.constructEquation(i_coeffs);
        }
        returnValue.push_back(func);
        for (int order = 0; order < i_order; order++)
        {
            func = func.differentiate();
            returnValue.push_back(func);
        }
        return returnValue;
    }
private:
    double m_time; // store the last time the coefficent was used to compute. not really needed.
    coefficients m_coeffs; // internal coefficients for this polynomial.
};
}
#endif