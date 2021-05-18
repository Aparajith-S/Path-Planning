/// \file
/// \brief
/// \author s.aparajith@live.com
/// \copyright None reserved. MIT license
/// 
/// 
#ifndef POLY_FUNCTION_H
#define POLY_FUNCTION_H
#include"trajectory.hpp"
namespace path_planner {
///  @class polyfunction class.
class function
{
public:
    explicit function() :
        m_time(-1.0),
        m_coeffs(vector<double>())
    {}
    virtual ~function() {}
    function(const function&) = default;
    function& operator=(const function&) = default;

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
    void constructEquation(const coefficients& coeffs)
    {
        this->m_coeffs = coeffs;
    }

    function differentiate(const coefficients& coeffs = coefficients())
    {
        function func;
        coefficients newCoeff = vector<double>();
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
    //returns the function and its N derivatives. default returns vector of {f, fdot,fdotdot}
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
    double m_time;
    coefficients m_coeffs;
};
}
#endif