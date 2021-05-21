#ifndef HAL_PIN_INTERFACE_H
#define HAL_PIN_INTERFACE_H

/// \author: Robert W. Ellenberg

#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>
#include <hal_types.h>

namespace machinekit_interfaces
{
// Resource wrapper for a read / write S32 HAL pin
template <typename T>
class HALPinHandle
{
public:
  HALPinHandle() = default;
  HALPinHandle(const std::string& name, T* val_ptr)
    : name_(name), pin_ptr_(val_ptr)
  {
  }

  // For calling from without external application
  T get() const
  {
    assert(pin_ptr_);
    return *pin_ptr_;
  }
  void set(T value)
  {
    assert(pin_ptr_);
    *pin_ptr_ = value;
  }

  std::string getName() const
  {
    return name_;
  }

private:
  std::string name_;
  T* pin_ptr_ = { nullptr };
};

typedef HALPinHandle<hal_s32_t> HALS32PinHandle;
typedef HALPinHandle<hal_u32_t> HALU32PinHandle;
typedef HALPinHandle<hal_bit_t> HALBitPinHandle;
// Can't use hal_float_t here because __attribute__((aligned(8))) causes
// warnings: warning: ignoring attributes on template argument ‘hal_float_t’
// {aka ‘double’}
typedef HALPinHandle<double> HALFloatPinHandle;

/** \brief Hardware interfaces to support reading/writing HAL pins */
typedef hardware_interface::HardwareResourceManager<HALS32PinHandle>
    HALS32PinInterface;
typedef hardware_interface::HardwareResourceManager<HALU32PinHandle>
    HALU32PinInterface;
typedef hardware_interface::HardwareResourceManager<HALBitPinHandle>
    HALBitPinInterface;
typedef hardware_interface::HardwareResourceManager<HALFloatPinHandle>
    HALFloatPinInterface;

}  // namespace machinekit_interfaces
#endif  // HAL_PIN_INTERFACE_H
