/**
 * @file    protocol.hpp
 * @author  syhanjin
 * @date    2026-02-24
 */
#pragma once
#include "crc.hpp"
#include "UartRxSync.hpp"
#include "RingBuffer.hpp"

// 上位机指令接收
// 协议 AA BB | CMD | 6 float | CRC16
using CRC16_Modbus = crc::CRCX<16, 0x8005, 0xFFFF, true, true, 0x0000>;

constexpr uint32_t HeaderLen = 2;
constexpr uint32_t FrameLen  = 29;

class PCProtocol final : public protocol::UartRxSync<HeaderLen, FrameLen>
{
public:
    explicit PCProtocol(UART_HandleTypeDef* huart) : UartRxSync(huart) {}

    struct Frame
    {
        uint32_t timestamp;
        struct Data
        {
            uint8_t cmd;
            uint8_t data[4 * 6];
        } data;
    };

    // libs::RingBuffer<>
    libs::RingBuffer<Frame, 10> rx_buffer_{};

    void timeAlignStart()
    {
        time_align_.started   = true;
        time_align_.counter   = 0;
        time_align_.delay_sum = 0;
    }
    void timeAlign(const uint32_t self_time, const float pc_time)
    {
        ++time_align_.counter;
        time_align_.delay_sum += pc_time - static_cast<float>(self_time);
        time_align_.total_time = self_time;
    }
    void timeAlignEnd()
    {
        time_align_.started = false;
        if (time_align_.counter <= 1)
            return;
        const float ave_delay = time_align_.delay_sum / static_cast<float>(time_align_.counter);
        timebase_             = ave_delay + transitionDelayMS();

        time_align_.finished = true;
    }
    [[nodiscard]] constexpr float transitionDelayMS() const
    {
        return FrameLen * 10 * 1000.0f / static_cast<float>(huart()->Init.BaudRate);
    }
    [[nodiscard]] bool isTimeAligning() const { return time_align_.started; }
    [[nodiscard]] bool isTimeAligned() const { return time_align_.finished; }

    [[nodiscard]] uint32_t pcTime2SelfTime(const float pc_time) const
    {
        return static_cast<uint32_t>(pc_time + 0.5f - timebase_);
    }

    [[nodiscard]] float selfTime2PCTime(const uint32_t self_time) const
    {
        return static_cast<float>(self_time) + timebase_;
    }

    [[nodiscard]] float getPCTime() const { return selfTime2PCTime(HAL_GetTick()); }

protected:
    static constexpr std::array<uint8_t, 2>     HEADER = { 0xAA, 0xBB };
    [[nodiscard]] const std::array<uint8_t, 2>& header() const override { return HEADER; }

    bool decode(const uint8_t data[27]) override;

private:
    struct
    {
        bool started  = false;
        bool finished = false;

        uint32_t counter    = 0;
        uint32_t total_time = 0;

        float delay_sum{ 0 };
    } time_align_;
    float timebase_{ 0 };
};

extern PCProtocol* pc_rx;

void Protocol_Init();