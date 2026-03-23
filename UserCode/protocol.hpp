/**
 * @file    protocol.hpp
 * @author  syhanjin
 * @date    2026-02-24
 */
#pragma once
#include "crc.hpp"
#include "UartRxSync.hpp"
#include "RingBuffer.hpp"

namespace Protocol
{

// 上位机指令接收
// 协议 AA BB | CMD | 6 float | CRC16
using CRC16_Modbus = crc::CRCX<16, 0x8005, 0xFFFF, true, true, 0x0000>;

constexpr uint32_t HeaderLen = 2;
constexpr uint32_t FrameLen  = 2 + 1 + 2 * 6 + 4 + 2;

class Clock
{
public:
    [[nodiscard]] uint32_t pcTime2SelfTime(const uint32_t pc_time) const
    {
        return static_cast<uint32_t>(static_cast<float>(pc_time) + 0.5f - offset_);
    }

    [[nodiscard]] uint32_t selfTime2PCTime(const uint32_t self_time) const
    {
        return static_cast<uint32_t>(static_cast<float>(self_time) + 0.5f + offset_);
    }

    [[nodiscard]] float pcTime2SelfTime(const float pc_time) const { return pc_time - offset_; }

    [[nodiscard]] float selfTime2PCTime(const float self_time) const { return self_time + offset_; }

    [[nodiscard]] uint32_t getPCTime() const { return selfTime2PCTime(HAL_GetTick()); }

    void align(const float self_time, const float pc_time)
    {
        const float delta = pc_time - self_time;

        offset_ = offset_ * (1 - alpha) + delta * alpha;
    }

private:
    float offset_{ 0 };

    constexpr static float alpha = 0.2;
};

class PCProtocol final : public protocol::UartRxSync<HeaderLen, FrameLen>
{
public:
    explicit PCProtocol(UART_HandleTypeDef* huart) : UartRxSync(huart) {}

    struct Frame
    {
        uint32_t rx_timestamp;
        uint8_t  cmd;
        uint8_t  data[2 * 6];
        uint32_t tx_timestamp;
        uint16_t crc16;
    };

    // libs::RingBuffer<>
    libs::RingBuffer<Frame, 10> rx_buffer_{};

    [[nodiscard]] float transitionDelayMS() const
    {
        return FrameLen * 10 * 1000.0f / static_cast<float>(huart()->Init.BaudRate);
    }

protected:
    static constexpr std::array<uint8_t, 2>     HEADER = { 0xAA, 0xBB };
    [[nodiscard]] const std::array<uint8_t, 2>& header() const override { return HEADER; }

    bool decode(const uint8_t data[19]) override;

    [[nodiscard]] uint32_t timeout() const override { return 250; }

private:
};

inline PCProtocol* pc_rx;
inline Clock*      clock_;

void init();
} // namespace Protocol