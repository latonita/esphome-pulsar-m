#include "object_locker.h"

namespace esphome {
namespace pulsar_m {

std::vector<void *> AnyObjectLocker::locked_objects_(5);
Mutex AnyObjectLocker::lock_;

};  // namespace pulsar_m
};  // namespace esphome
