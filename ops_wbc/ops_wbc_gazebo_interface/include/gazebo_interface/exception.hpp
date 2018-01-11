#ifndef __OPS_WBC_GAZEBO_INTERFACE_EXCEPTION_HPP
#define __OPS_WBC_GAZEBO_INTERFACE_EXCEPTION_HPP

#include <sstream>

namespace ops_wbc_gazebo_interface
{

  /// \brief ahl_gazebo_if::Exception
  class Exception
  {
  public:
    /// \brief Constructor
    /// \param src name of function which threw exception
    /// \param msg description of exception
    explicit Exception(const std::string& src, const std::string& msg) throw()
      : src_(src), msg_(msg)
    {}

    /// \brief get description of exception
    /// \return description of exception
    const char* what() const throw()
    {
      std::stringstream ss;
      ss << "ahl_gazebo_if::Exception occurred." << std::endl
         << "  src : " << src_ << std::endl
         << "  msg : " << msg_;

      return ss.str().c_str();
    }

  private:
    /// name of function which threw exception
    std::string src_;
    /// description of exception
    std::string msg_;
  };

} // namespace ahl_gazebo_if

#endif // __OPS_WBC_GAZEBO_INTERFACE_EXCEPTION_HPP

