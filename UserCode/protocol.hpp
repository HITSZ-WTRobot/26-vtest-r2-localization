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

class PCProtocol final : public protocol::UartRxSync<2, 29>
{
public:
    explicit PCProtocol(UART_HandleTypeDef* huart) : UartRxSync(huart) {}

    struct Frame
    {
        uint8_t cmd;
        uint8_t data[4 * 6];
    };

    // libs::RingBuffer<>
    libs::RingBuffer<Frame, 10> rx_buffer_{};

protected:
    static constexpr std::array<uint8_t, 2>     HEADER = { 0xAA, 0xBB };
    [[nodiscard]] const std::array<uint8_t, 2>& header() const override { return HEADER; }
    bool                                        decode(const uint8_t data[27]) override;
};

extern PCProtocol* pc_rx;

void Protocol_Init();