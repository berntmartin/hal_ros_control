#ifndef GENERIC_INTERFACE_H
#define GENERIC_INTERFACE_H

/// \author: Robert W. Ellenberg

#pragma once


#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace machinekit_interfaces
{

// Resource wrapper for a read / write S32 HAL pin
class GenericInt32Handle
{
public:
  GenericInt32Handle() = default;
  GenericInt32Handle( const std::string &name, int *val_ptr) : name_(name), pin_ptr_(val_ptr)
  {
  }

  // For calling from without external application
  int get() const {
      assert(pin_ptr_);
      return *pin_ptr_;
  }
  void set(int value)  {
      assert(pin_ptr_);
      *pin_ptr_ = value;
  }

  std::string getName() const {return name_;}

private:
  std::string name_;
  int *pin_ptr_ = {nullptr};
};


/** \brief Hardware interface to support reading the state of a force-torque sensor. */
class GenericInt32Interface : public hardware_interface::HardwareResourceManager<GenericInt32Handle> {};

}
#endif // GENERIC_INTERFACE_H
