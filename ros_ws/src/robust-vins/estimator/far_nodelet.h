#pragma once

#include <memory>
#include <nodelet/nodelet.h>

namespace FAR {

// forward declarations
class FARInterface;

class FARNodelet : public nodelet::Nodelet
{
public:
  FARNodelet() = default;
  virtual ~FARNodelet();

 private:
  virtual void onInit();

  std::unique_ptr<FARInterface> far_interface_;
};

} // namespace FAR
