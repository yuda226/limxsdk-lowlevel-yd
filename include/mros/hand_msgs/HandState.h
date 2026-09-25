#ifndef _LIMX_SDK_COMPAT_MROS_HAND_MSGS_HANDSTATE_H_
#define _LIMX_SDK_COMPAT_MROS_HAND_MSGS_HANDSTATE_H_

#include <memory>

#include "limxsdk/msg/hand_msgs/HandState.h"

namespace mros
{
namespace hand_msgs
{
using HandState = ::limxsdk::msg::hand_msgs::HandState;
using HandStatePtr = std::shared_ptr<HandState>;
using HandStateConstPtr = std::shared_ptr<const HandState>;
}  // namespace hand_msgs
}  // namespace mros

#endif  // _LIMX_SDK_COMPAT_MROS_HAND_MSGS_HANDSTATE_H_
