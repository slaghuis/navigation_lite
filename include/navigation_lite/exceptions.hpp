#ifndef NAVIGATION_LITE__EXCEPTIONS_HPP_
#define NAVIGATION_LITE__EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>
#include <memory>

namespace navigation_lite
{
class ControllerException : public std::runtime_error
{
public:
  explicit ControllerException(const std::string description)
  : std::runtime_error(description) {}
  using Ptr = std::shared_ptr<ControllerException>;
};
  

} // namespace navigation_lite

#endif // NAVIGATION_LITE__EXCEPTIONS_HPP_
