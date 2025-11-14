/**
 * -------------------------------------------------------------------------------------------------
 * 
 * Copyright 2025 Minkyu Kil
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * @file    serial_packet.hpp
 * @brief   Henes T870 serial packet byte name wrapper and factors
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef T870_SERIAL__SERIAL_PACKET_HPP_
#define T870_SERIAL__SERIAL_PACKET_HPP_

namespace t870_serial
{
    // Henes T870 serial tx packet index mapping
    namespace TX
    {
        enum ByteName
        {
            STX_S          = 0,
            STX_T          = 1,
            STX_X          = 2,
            CONTROL_MODE   = 3,
            EMERGENCY_STOP = 4,
            GEAR           = 5,
            SPEED_RAW_0    = 6,
            SPEED_RAW_1    = 7,
            STEERING_100_0 = 8,
            STEERING_100_1 = 9,
            HEARTBEAT      = 10,
            ETX_0          = 11,
            ETX_1          = 12,
            PACKET_SIZE    = 13

        }; // enum ByteName

    } // namespace TX

    // Henes T870 serial rx packet index mapping
    namespace RX
    {
        enum ByteName
        {
            STX_S          = 0,
            STX_T          = 1,
            STX_X          = 2,
            CONTROL_MODE   = 3,
            EMERGENCY_STOP = 4,
            GEAR           = 5,
            SPEED_RAW_0    = 6,
            SPEED_RAW_1    = 7,
            STEERING_100_0 = 8,
            STEERING_100_1 = 9,
            ENCODER_0      = 10,
            ENCODER_1      = 11,
            ENCODER_2      = 12,
            ENCODER_3      = 13,
            HEARTBEAT      = 14,
            ETX_0          = 15,
            ETX_1          = 16,
            PACKET_SIZE    = 17

        }; // enum ByteName

    } // namespace RX

    // Speed(m/s) -> Raw byte command
    static constexpr double MPS2BYTE {1.0}; // TODO

    // Steering(rad) -> Raw byte command
    static constexpr double RAD2BYTE {-5729.57795131};

    // Raw byte command -> Speed (m/s)
    static constexpr double BYTE2MPS {1.0}; // TODO

    // Raw byte command -> Steering (rad)
    static constexpr double BYTE_STEERING_CENTER {500.0};
    static constexpr double BYTE2RAD {-0.00116355283};

} // namespace t870_serial

#endif // T870_SERIAL__SERIAL_PACKET_HPP_