/*
 * TalonUtils.cpp
 *
 *  Created on: Jan 14, 2020
 *      Author: PHS_User
 */

#include "frc2135/TalonUtils.h"

#include <chrono>
#include <frc/RobotBase.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

namespace frc2135
{
    TalonUtils::TalonUtils()
    {
        // Auto-generated constructor stub
    }

    TalonUtils::~TalonUtils()
    {
        // Auto-generated destructor stub
    }

    bool TalonUtils::TalonCheck(WPI_BaseMotorController &talon, const char *subsystem, const char *name)
    {
        int i;
        int deviceID = 0;
        int fwVersion = 0;
        bool talonValid = false;
        ErrorCode error = OKAY;

        // Configure subsystem

        // Display Talon firmware versions
        deviceID = talon.GetDeviceID();
        if ((error = talon.GetLastError()) != OKAY)
        {
            spdlog::error("{} Motor {} GetDeviceID error - {}", subsystem, name, error);
            return (error == OKAY);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        for (i = 0; i < m_retries; i++)
        {
            fwVersion = talon.GetFirmwareVersion();
            if ((error = talon.GetLastError()) != OKAY)
            {
                spdlog::error("{} Motor {} ID {} GetFirmwareVersion error - {}", subsystem, name, deviceID, error);
                return (error == OKAY);
            }
            if (fwVersion >= m_reqVersion)
            {
                talonValid = true;
                break;
            }
            else
            {
                spdlog::warn(
                    "{} Motor {} ID {} Incorrect FW version {}.{} expected {}.{}",
                    subsystem,
                    name,
                    deviceID,
                    fwVersion / 256,
                    fwVersion & 0xff,
                    m_reqVersion / 256,
                    m_reqVersion & 0xff);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        if (talonValid)
        {
            // Initialize Talon to all factory defaults
            if ((error = talon.ConfigFactoryDefault(kCANTimeout)) != OKAY)
            {
                spdlog::error("{} Motor {} ID {} ConfigFactoryDefault error - {}", subsystem, name, deviceID, error);
            }
            else
            {
                spdlog::info(
                    "{} Motor {} ID {} ver {}.{} is RESPONSIVE and INITIALIZED (error {})",
                    subsystem,
                    name,
                    deviceID,
                    fwVersion / 256,
                    fwVersion & 0xff,
                    error);
            }
        }
        else
        {
            spdlog::error(
                "{} Motor {} ID {} ver {}.{} is UNRESPONSIVE, (error {})",
                subsystem,
                name,
                deviceID,
                fwVersion / 256,
                fwVersion & 0xff,
                error);
        }

        return talonValid;
    }

    void TalonUtils::TalonFaultDump(const char *talonName, WPI_BaseMotorController &talon)
    {
        int fwVersion = 0;
        ErrorCode error = OKAY;
        Faults faults;
        StickyFaults stickyFaults;

        // Print out Talon faults and clear sticky ones
        spdlog::info("Talon -------------- {}", talonName);

        // Check Talon by getting device ID and validating firmware versions
        talon.GetDeviceID();
        if ((error = talon.GetLastError()) != OKAY)
        {
            spdlog::error("Motor {} GetDeviceID error - {}", talonName, error);
            return;
        }

        fwVersion = talon.GetFirmwareVersion();
        if ((error = talon.GetLastError()) != OKAY)
        {
            spdlog::error("Motor {} GetFirmwareVersion error - {}", talonName, error);
            return;
        }
        if (fwVersion != m_reqVersion)
        {
            spdlog::warn(
                "Motor {} Incorrect FW version {}.{} expected {}.{}",
                talonName,
                fwVersion / 256,
                fwVersion & 0xff,
                m_reqVersion / 256,
                m_reqVersion & 0xff);
            return;
        }

        // Now the Talon has been validated
        talon.GetFaults(faults);
        talon.GetStickyFaults(stickyFaults);
        talon.ClearStickyFaults(100);

        if (faults.HasAnyFault())
        {
            // Do we need this
            spdlog::warn("At least one fault below");
            spdlog::warn("faults: {}", faults.ToString());
        }
        else
        {
            spdlog::info("NO Talon FX active faults detected");
        }

        if (stickyFaults.HasAnyFault())
        {
            spdlog::warn("At least one STICKY fault below");
            spdlog::warn("sticky faults: {}", stickyFaults.ToString());
        }
        else
        {
            spdlog::info("NO Talon FX sticky faults detected");
        }
    }

    ///////////////////////////////////////////////////////////////////////////////
    //
    //    Pigeon IMU
    //
    bool TalonUtils::PigeonIMUInitialize(PigeonIMU &pigeonPtr)
    {
        int i;
        int retries = 5;
        int deviceID = 0;
        int pigeonVersion = 0;
        bool pigeonValid = false;
        ErrorCode error = OKAY;
        char subsystem[] = "DT";
        char name[] = "Pigeon IMU";

        // Display Pigeon IMU firmware versions
        deviceID = pigeonPtr.GetDeviceNumber();
        if (!frc::RobotBase::IsSimulation() && ((error = pigeonPtr.GetLastError()) != OKAY))
        {
            spdlog::error("{} {} GetDeviceNumber error - {}", subsystem, name, error);
            return (error == OKAY);
        }

        for (i = 0; i < retries; i++)
        {
            pigeonVersion = pigeonPtr.GetFirmwareVersion();
            if (!frc::RobotBase::IsSimulation() && ((error = pigeonPtr.GetLastError()) != OKAY))
            {
                spdlog::error("{} {} ID {} GetFirmwareVersion error - {}", subsystem, name, deviceID, error);
                return (error == OKAY);
            }
            else if (pigeonVersion >= m_reqPigeonVer)
            {
                pigeonValid = true;
                break;
            }
            else
            {
                spdlog::warn(
                    "{} {} ID {} Incorrect FW version {}.{} expected {}.{}",
                    subsystem,
                    name,
                    deviceID,
                    pigeonVersion / 256,
                    pigeonVersion & 0xff,
                    m_reqPigeonVer / 256,
                    m_reqPigeonVer & 0xff);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (pigeonValid)
        {
            // Initialize Pigeon IMU to all factory defaults
            if ((error = pigeonPtr.ConfigFactoryDefault(kCANTimeout)) != OKAY)
            {
                spdlog::error("{} {} ID {} ConfigFactoryDefault error - {}", subsystem, name, deviceID, error);
                pigeonValid = false;
            }

            double headingDeg = pigeonPtr.GetFusedHeading();
            bool angleIsGood = (pigeonPtr.GetState() == PigeonIMU::Ready) ? true : false;
            spdlog::info(
                "{} {} ID {} fused heading angle is {} degrees {}",
                subsystem,
                name,
                deviceID,
                headingDeg,
                (angleIsGood) ? "VALID" : "NOTREADY");

            pigeonPtr.SetYaw(0.0, kCANTimeout);
            if ((error = pigeonPtr.GetLastError()) != OKAY)
            {
                spdlog::error("{} {} ID {} SetFusedHeading error - {}", subsystem, name, deviceID, error);
                pigeonValid = false;
            }

            pigeonPtr.SetFusedHeading(0.0, kCANTimeout);
            if ((error = pigeonPtr.GetLastError()) != OKAY)
            {
                spdlog::error("{} {} ID {} SetYaw error - {}", subsystem, name, deviceID, error);
                pigeonValid = false;
            }

            if (pigeonValid)
            {
                spdlog::info(
                    "{} {} ID {} ver {}.{} is RESPONSIVE and INITIALIZED (error {})",
                    subsystem,
                    name,
                    deviceID,
                    pigeonVersion / 256,
                    pigeonVersion & 0xff,
                    error);
            }
        }
        else
        {
            spdlog::error(
                "{} {} ID {} ver {}.{} is UNRESPONSIVE, (error {})",
                subsystem,
                name,
                deviceID,
                pigeonVersion / 256,
                pigeonVersion & 0xff,
                error);
            pigeonValid = false;
        }

        return pigeonValid;
    }

    void TalonUtils::PigeonIMUFaultDump(const char *pigeonName, PigeonIMU &pigeonPtr)
    {
        int deviceID = 0;
        int fwVersion = 0;
        ErrorCode error = OKAY;
        PigeonIMU_Faults faults;
        PigeonIMU_StickyFaults stickyFaults;

        // Print out PigeonIMU faults and clear sticky ones
        spdlog::info("PigeonIMU -------------- {}", pigeonName);

        // Check PigeonIMU by getting device ID and validating firmware versions
        deviceID = pigeonPtr.GetDeviceNumber();
        if ((error = pigeonPtr.GetLastError()) != OKAY)
        {
            spdlog::error("PigeonIMU Gyro GetDeviceID error - {}", error);
            return;
        }

        fwVersion = pigeonPtr.GetFirmwareVersion();
        if ((error = pigeonPtr.GetLastError()) != OKAY)
        {
            spdlog::error("PigeonIMU Gyro GetFirmwareVersion error - {}", error);
            return;
        }
        if (fwVersion != m_reqVersion)
        {
            spdlog::warn(
                "PigeonIMU Gyro Incorrect FW version {}.{} expected {}.{}",
                fwVersion / 256,
                fwVersion & 0xff,
                m_reqVersion / 256,
                m_reqVersion & 0xff);
            return;
        }

        // Now the PigeonIMU has been validated
        pigeonPtr.GetFaults(faults);
        pigeonPtr.GetStickyFaults(stickyFaults);
        pigeonPtr.ClearStickyFaults(100);

        if (faults.HasAnyFault())
        {
            spdlog::error("PigeonIMU ID {} has a FAULT - {}", deviceID, faults.ToBitfield());
        }
        else
        {
            spdlog::info("NO PigeonIMU sticky faults detected");
        }
    }

    void TalonUtils::CheckError(ErrorCode errorCode, std::string message)
    {
        if (errorCode != OK)
            spdlog::error("Error code: {} Message: {}", errorCode, message);
    }

} /* namespace frc2135 */
