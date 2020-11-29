/*
 * TalonUtils.h
 *
 *  Created on: Jan 14, 2020
 *      Author: PHS_User
 */

#pragma once

#include <ctre/Phoenix.h>

namespace frc2135
{

class TalonUtils
{

private:
    static const int    kCANTimeout = 30;               // CAN timeout in msec to wait for response
    static const int    m_retries = 4;                  // Number of version check attempts

    static const int    m_reqVersion = ((20*256)+2);    // Talon version is 20.2

public:
    TalonUtils();
    virtual ~TalonUtils();
    static bool TalonCheck(std::shared_ptr<WPI_BaseMotorController> talon, const char *subsystem, const char *name);
    static void TalonFaultDump(const char *talonName, std::shared_ptr<WPI_BaseMotorController> talonPtr);
    static void PigeonIMUFaultDump(const char *pigeonName, std::shared_ptr<PigeonIMU> pigeonPtr);
};

} /* namespace frc2135 */

