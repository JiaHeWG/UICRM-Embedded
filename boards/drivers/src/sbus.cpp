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

#include "sbus.h"

#include <cmath>
#include <cstring>

#include "bsp_error_handler.h"
#include "utils.h"

/* iA6B SBUS 帧中摇杆通道中位点（11bit 无符号 raw 的中位） */
#define SBUS_RC_ROCKER_MID  1024

namespace remote {

    // SBUS单数据包RAW输出写死25字节，此处需要实机验证
    SBUS::SBUS(UART_HandleTypeDef* huart) : bsp::UART(huart) {
        SetupRx(FRAME_SIZE);
    }

    int16_t SBUS::MapTo660(const int16_t val) const {
        //   正半轴分母 783，负半轴分母 784，四舍五入 +0.5，参考开源拟合平滑取值
        if (val >= 0) {
            return (int16_t)floorf((660.0f / 783.0f) * (float)val + 0.5f);
        } else {
            return (int16_t)floorf((660.0f / 784.0f) * (float)val + 0.5f);
        }
    }

    void SBUS::RxCompleteCallback() {
        Heartbeat();
        uint8_t* data = nullptr;
        // Abnormal data processing
        if (this->Read<true>(&data) != FRAME_SIZE) {
            return;
        }
        if (data[0] != SBUS_START_BYTE || data[24] != SBUS_END_BYTE) {
            return;
        }
        // 10 of 11bit channels for sbus decoding, from i6x original source
        // TODO:这里需要注意，SBUS与原有DBUS协议栈有很大不同，需要完全修改计算逻辑，我还没来得及回溯底层bsp_uart中有没有做dbus/sbus解码，但是这里原有写法没做，我就默认直接拉raw了
        int16_t raw[10];
        raw[0] = (int16_t)(((data[1]                       ) & 0x07FF) - SBUS_RC_ROCKER_MID);
        raw[1] = (int16_t)((((data[2] >> 3) | (data[3] << 5)) & 0x07FF) - SBUS_RC_ROCKER_MID);
        raw[2] = (int16_t)((((data[3] >> 6) | (data[4] << 2) | (data[5] << 10)) & 0x07FF) - SBUS_RC_ROCKER_MID);
        raw[3] = (int16_t)((((data[5] >> 1) | (data[6] << 7)) & 0x07FF) - SBUS_RC_ROCKER_MID);
        raw[4] = (int16_t)((((data[6] >> 4) | (data[7] << 4)) & 0x07FF) - SBUS_RC_ROCKER_MID);
        raw[5] = (int16_t)((((data[7] >> 7) | (data[8] << 1) | (data[9] << 9)) & 0x07FF) - SBUS_RC_ROCKER_MID);
        raw[6] = (int16_t)((((data[9] >> 2) | (data[10] << 6)) & 0x07FF) - SBUS_RC_ROCKER_MID);
        raw[7] = (int16_t)((((data[10] >> 5) | (data[11] << 3)) & 0x07FF) - SBUS_RC_ROCKER_MID);
        raw[8] = (int16_t)(((data[12] | (data[13] << 8)) & 0x07FF) - SBUS_RC_ROCKER_MID);
        raw[9] = (int16_t)((((data[13] >> 3) | (data[14] << 5)) & 0x07FF) - SBUS_RC_ROCKER_MID);

        // 摇杆通道重映射，平滑拟合
        for (uint8_t i = 0; i < 6; ++i) {
            ch[i] = MapTo660(raw[i]);
        }

        // sw[0]、sw[1]、sw[3]：两档拨杆（<0 视为上，>=0 视为下）
        sw[0] = (raw[6] < 0) ? SW_2POS_UP : SW_2POS_DOWN;
        sw[1] = (raw[7] < 0) ? SW_2POS_UP : SW_2POS_DOWN;
        sw[3] = (raw[9] < 0) ? SW_2POS_UP : SW_2POS_DOWN;

        // sw[2]：三档拨杆
        if (raw[8] < -SW_3POS_THRESHOLD) {
            sw[2] = SW_UP;
        } else if (raw[8] > SW_3POS_THRESHOLD) {
            sw[2] = SW_DOWN;
        } else {
            sw[2] = SW_MID;
        }

        // SBUS标志位解码，TODO:待验证
        const uint8_t flag = data[23];
        frame_lost = (flag >> 2) & 0x01;
        failsafe   = (flag >> 3) & 0x01;
        timestamp = GetLastUptime();
    }

} /* namespace remote */