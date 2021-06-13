/// \file logging.h
/// \brief logger object to handle logging with a flag
/// \author s.aparajith@live.com
/// \date 13.06.2021
/// \copyright None reserved. MIT license
#ifndef LOGGING_H
#define LOGGING_H
#include<string>
#include<iostream>
#include<sstream>
#define log_on (0)
class logger
{
  public:
  logger()=default;
  ~logger(){}

  /// \brief << operator for const char and strings
  std::ostream& operator<<(const std::string& stream)
  {
    std::stringstream sstr;
    sstr<<stream;
    m_internal<<stream;
    #if log_on == 1
	    std::cout<<stream<<std::endl;
    #endif
    return m_internal;
  }

  /// \brief << operator for integers
  std::ostream& operator<<(const int& stream)
  {
    std::stringstream sstr;
    sstr<<stream;
    m_internal<<stream;
    #if log_on == 1
	    std::cout<<stream<<std::endl;
    #endif
    return m_internal;
  }

  /// \brief << operator for floating point IEEE754 double precision
  std::ostream& operator<<(const double& stream)
  {
    std::stringstream sstr;
    sstr<<stream;
    m_internal<<stream;
    #if log_on == 1
	    std::cout<<stream<<std::endl;
    #endif
    return m_internal;
  }

  /// \brief << operator for float IEEE754 single precision
  std::ostream& operator<<(const float& stream)
  {
    std::stringstream sstr;
    sstr<<stream;
    m_internal<<stream;
    #if log_on == 1
	    std::cout<<stream<<std::endl;
    #endif
    return m_internal;
  }

  /// @brief flush the internal buffer
  void flush()
  {
    m_internal.flush();
  }

  std::stringstream m_internal; // internal buffer
};

#endif