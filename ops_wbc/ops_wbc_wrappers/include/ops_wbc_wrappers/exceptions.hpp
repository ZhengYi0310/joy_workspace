#ifndef __OPS_WBC_WRAPPERS_EXCEPTIONS_HPP
#define __OPS_WBC_WRAPPERS_EXCEPTIONS_HPP

#include <sstream>

namespace ops_wbc_wrappers
{
  
  class Exception
  {
  public:
    explicit Exception(const std::string& src, const std::string& msg) throw()
      : src_(src), msg_(msg)
    {
    }
    
    const char* what() const throw()
    {
      std::stringstream ss;
      ss << "OPS_WBC_WRAPPERS::Exception was thrown." << std::endl
         << "  src : " << src_ << std::endl
         << "  msg : " << msg_;

      return ss.str().c_str();
    }

  private:    
    std::string src_;
    std::string msg_;
  };

  class FatalException
  {
  public:    
    explicit FatalException(const std::string& src, const std::string& msg) throw()
      : src_(src), msg_(msg)
    {
    }
    
    const char* what() const throw()
    {
      std::stringstream ss;
      ss << "OPS_WBC_WRAPPERS::FatalException was thrown." << std::endl
         << "  src : " << src_ << std::endl
         << "  msg : " << msg_;
    }

  private:
    std::string src_;
    std::string msg_;
  };
  
}

#endif /* __OPS_WBC_WRAPPERs_EXCEPTIONS_HPP */
