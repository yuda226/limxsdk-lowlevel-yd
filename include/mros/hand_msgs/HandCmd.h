#ifndef _LIMX_SDK_COMPAT_MROS_HAND_MSGS_HANDCMD_H_
#define _LIMX_SDK_COMPAT_MROS_HAND_MSGS_HANDCMD_H_

#include <memory>

#include "limxsdk/msg/hand_msgs/HandCmd.h"

namespace mros
{
namespace hand_msgs
{
using HandCmd = ::limxsdk::msg::hand_msgs::HandCmd;
using HandCmdPtr = std::shared_ptr<HandCmd>;
using HandCmdConstPtr = std::shared_ptr<const HandCmd>;
}  // namespace hand_msgs
}  // namespace mros

#endif  // _LIMX_SDK_COMPAT_MROS_HAND_MSGS_HANDCMD_H_
