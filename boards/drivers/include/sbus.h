/*###########################################################
 # Copyright (c) 2023-2026. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#pragma once

#include <stdint.h>

#include "bsp_uart.h"
#include "connection_driver.h"

namespace remote {

    /**
     * @brief SBUS 遥控器接收类
     * @note  用于支持 SBUS 的接收机（FS-iA6B）
     *
     * 数据来源：FS-i6X 遥控器 + FS-iA6B 接收机 SBUS 输出
     *   ch[0] ~ ch[5] : 6 个摇杆通道（映射到 -660~660）
     *   sw[0] ~ sw[3] : 4 个拨杆通道（sw[0]/sw[1]/sw[3] 两档，sw[2] 三档）
     *   frame_lost / failsafe : 丢帧与失控标志
     * @note 遥控器的样式(FS-i6X)
     * @note the style of the remote
     *      C4(   )C5
     * SW1* SW2* *SW3 *SW4
     *   C2-^       ^-C1
     * C3-<   >+ -<   >+C0
     *     +v       v+
     */

    // TODO: 这里只按预期效果（C板直通UART3）做了理论重构，大量采用原有i6x.c/.h逻辑实现，需要上机验证。
    // TODO: 同时需要注意，i6X遥控器的按键/摇杆布局与我们现有的DT7方案差距较大，考虑后期通过映射等方式尽量做兼容，避免换控要改上游程序
    class SBUS : public bsp::UART, public driver::ConnectionDriver {
    public:
        explicit SBUS(UART_HandleTypeDef* huart);

        // 线性重映射，平滑拟合
        int16_t MapTo660(const int16_t val) const;

        void RxCompleteCallback() override final;

        // 帧长度定义
        static constexpr uint16_t FRAME_SIZE      = 25;
        static constexpr uint16_t SBUS_START_BYTE = 0x0F;
        static constexpr uint16_t SBUS_END_BYTE   = 0x00;

        static constexpr int16_t ROCKER_MIN = -660;
        static constexpr int16_t ROCKER_MAX =  660;

        // 拨杆定义
        static constexpr int8_t SW_UP         =  1;
        static constexpr int8_t SW_MID        =  0;
        static constexpr int8_t SW_DOWN       = -1;
        // 拨杆分两档/三档，需要分开取值，阈值不同
        static constexpr int8_t SW_2POS_UP    =  1;
        static constexpr int8_t SW_2POS_DOWN  =  0;
        // 三档拨杆阈值
        static constexpr int16_t SW_3POS_THRESHOLD = 200;

        // 解包存储位
        volatile int16_t ch[6];    // 摇杆，-660~660
        volatile int8_t  sw[4];    // 拨杆，-1/0/1

        volatile uint8_t frame_lost;   // 丢帧标志（SBUS flag bit2）
        volatile uint8_t failsafe;     // 失控标志（SBUS flag bit3）

        volatile uint32_t timestamp;
    };

} /* namespace remote */