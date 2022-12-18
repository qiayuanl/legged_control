//
// Created by qiayuan on 2021/11/5.
//
#pragma once
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace legged {
class ContactSensorHandle {
 public:
  ContactSensorHandle() = default;

  ContactSensorHandle(const std::string& name, const bool* isContact) : name_(name), isContact_(isContact) {
    if (isContact == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. isContact pointer is null.");
    }
  }

  std::string getName() const { return name_; }

  bool isContact() const {
    assert(isContact_);
    return *isContact_;
  }

 private:
  std::string name_;

  const bool* isContact_ = {nullptr};
};

class ContactSensorInterface
    : public hardware_interface::HardwareResourceManager<ContactSensorHandle, hardware_interface::DontClaimResources> {};

}  // namespace legged
