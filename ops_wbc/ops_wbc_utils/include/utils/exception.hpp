#ifndef __OPS_WBC_UTILS_EXCEPTION_HPP
#define __OPS_WBC_UTILS_EXCEPTION_HPP

#include <sstream>

namespace ops_wbc_utils
{

  class Exception
  {
  public:
    explicit Exception(const std::string& src, const std::string& msg) throw()
      : src_(src), msg_(msg)
    {
    }

    std::string what() throw()
    {
      std::stringstream msg;
      msg << "ops_wbc_utils::Exception was thrown." << std::endl
          << "  src : " << src_ << std::endl
          << "  msg : " << msg_;

      return msg.str();
    }

  private:
    std::string src_;
    std::string msg_;
  };

} 

#endif // __OPS_WBC_UTILS_EXCEPTION_HPP