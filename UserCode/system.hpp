/**
 * @file    system.hpp
 * @author  syhanjin
 * @date    2026-03-16
 */
#pragma once

namespace System
{

// 初始化过程变量
namespace Init
{
inline bool             posReceived = false;
inline chassis::Posture pos{};
// hook
extern void initPosReceived();

inline bool inited()
{
    return posReceived;
}
} // namespace Init

} // namespace System