// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "RobotContainer.h"
#include "frc2135/RobotConfig.h"
#include "frc2135/TalonUtils.h"
#include "subsystems/LED.h"

#include <frc/RobotController.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "subsystems/Shooter.h"

#include <frc/smartdashboard/SmartDashboard.h>

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

Shooter::Shooter()
{
    SetName("Shooter");
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    SetSubsystem("Shooter");

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    // Validate Talon FX controllers, initialize and display firmware versions
    m_talonValidSH11 = frc2135::TalonUtils::TalonCheck(m_motorSH11, "SH", "SH11");
    frc::SmartDashboard::PutBoolean("HL_SH11Valid", m_talonValidSH11);

    // Initialize Variables
    frc2135::RobotConfig *config = frc2135::RobotConfig::GetInstance();

    config->GetValueAsDouble("SH_FlywheelPidKf", m_flywheelPidKf, 0.0475);
    config->GetValueAsDouble("SH_FlywheelPidKp", m_flywheelPidKp, 0.05);
    config->GetValueAsDouble("SH_FlywheelPidKi", m_flywheelPidKi, 0.0);
    config->GetValueAsDouble("SH_FlywheelPidKd", m_flywheelPidKd, 0.0);
    config->GetValueAsDouble("SH_FlywheelNeutralDeadband", m_flywheelNeutralDeadband, 0.004);
    config->GetValueAsDouble("SH_FlywheelLowerHubTargetRPM", m_flywheelLowerHubTargetRPM, 1450.0);
    config->GetValueAsDouble("SH_FlywheelUpperHubTargetRPM", m_flywheelUpperHubTargetRPM, 3000.0);

    config->GetValueAsDouble("SH_ToleranceRPM", m_toleranceRPM, 200.0);

    frc::SmartDashboard::PutNumber("SH_FlywheelPidKf", m_flywheelPidKf);
    frc::SmartDashboard::PutNumber("SH_FlywheelPidKp", m_flywheelPidKp);
    frc::SmartDashboard::PutNumber("SH_FlywheelPidKi", m_flywheelPidKi);
    frc::SmartDashboard::PutNumber("SH_FlywheelPidKd", m_flywheelPidKd);
    frc::SmartDashboard::PutNumber("SH_FlywheelLowerHubTargetRPM", m_flywheelLowerHubTargetRPM);
    frc::SmartDashboard::PutNumber("SH_FlywheelUpperHubTargetRPM", m_flywheelUpperHubTargetRPM);

    frc::SmartDashboard::PutNumber("SH_ToleranceRPM", m_toleranceRPM);

    if (m_talonValidSH11)
    {
        // Set motor directions
        // Turn on Coast mode
        m_motorSH11.SetInverted(true);
        m_motorSH11.SetNeutralMode(NeutralMode::Coast);
        m_motorSH11.SetSafetyEnabled(false);

        // Enable voltage compensation
        m_motorSH11.ConfigNeutralDeadband(m_flywheelNeutralDeadband, kCANTimeout);
        m_motorSH11.ConfigPeakOutputReverse(0.0, kCANTimeout);

        m_motorSH11.ConfigSupplyCurrentLimit(m_supplyCurrentLimits);
        m_motorSH11.ConfigStatorCurrentLimit(m_statorCurrentLimits);

        // Configure sensor settings
        m_motorSH11.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, kPidIndex, kCANTimeout);
        m_motorSH11.SetSensorPhase(true);
        m_motorSH11.SetSelectedSensorPosition(0, kPidIndex, kCANTimeout);

        // Configure Velocity PIDF settings
        m_motorSH11.Config_kF(kSlotIndex, m_flywheelPidKf, kCANTimeout);
        m_motorSH11.Config_kP(kSlotIndex, m_flywheelPidKp, kCANTimeout);
        m_motorSH11.Config_kI(kSlotIndex, m_flywheelPidKi, kCANTimeout);
        m_motorSH11.Config_kD(kSlotIndex, m_flywheelPidKd, kCANTimeout);
        m_motorSH11.SelectProfileSlot(kSlotIndex, kPidIndex);

        m_motorSH11.Set(ControlMode::Velocity, 0.0);
    }

    Initialize();
}

void Shooter::Periodic()
{
    RobotContainer *robotContainer = RobotContainer::GetInstance();
    // Put code here to be run every loop

    if (m_talonValidSH11)
    {
        m_flywheelCurrentRPM =
            m_flywheelFilter.Calculate(NativeToFlywheelRPM(m_motorSH11.GetSelectedSensorVelocity(kPidIndex)));
    }

    frc::SmartDashboard::PutNumber("SH_FlywheelRPM", m_flywheelCurrentRPM);

    if (m_state != SHOOTERSPEED_STOP)
    {
        if (!IsAtDesiredRPM())
        {
            robotContainer->m_led.SetShooterColor(LED::LEDCOLOR_BLUE);
        }
        else
        {
            robotContainer->m_led.SetShooterColor(LED::LEDCOLOR_GREEN);
        }
    }
    else
    {
        robotContainer->m_led.SetShooterColor(LED::LEDCOLOR_OFF);
    }

    double currentSH11 = 0.0;

    if (m_talonValidSH11)
    {
        currentSH11 = m_motorSH11.GetOutputCurrent();
    }

    frc::SmartDashboard::PutNumber("SH_Current_SH11", currentSH11);

    if (m_motorSH11.HasResetOccurred())
    {
        m_resetCountSH11 += 1;
        frc::SmartDashboard::PutNumber("HL_Reset_SH11", m_resetCountSH11);
    }
}

void Shooter::SimulationPeriodic()
{
    // This method will be called once per scheduler run when in simulation
    TalonFXSimCollection &motorSim(m_motorSH11.GetSimCollection());

    /* Pass the robot battery voltage to the simulated Talon FXs */
    motorSim.SetBusVoltage(frc::RobotController::GetInputVoltage());

    m_flywheelSim.SetInputVoltage(motorSim.GetMotorOutputLeadVoltage() * 1_V);

    m_flywheelSim.Update(20_ms);

    motorSim.SetIntegratedSensorVelocity(FlywheelRPMToNative(
        kFlywheelGearRatio * m_flywheelSim.GetAngularVelocity().to<double>() * 60 / (2 * wpi::numbers::pi)));
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

// Put methods for controlling this subsystem
// here. Call these from Commands.

void Shooter::Initialize(void)
{
    spdlog::info("SH Init");
    SetShooterSpeed(SHOOTERSPEED_STOP);
}

void Shooter::FaultDump(void)
{
    //    Dump all Talon faults
    if (m_talonValidSH11)
        frc2135::TalonUtils::TalonFaultDump("SH 11", m_motorSH11);
}

double Shooter::FlywheelRPMToNative(double rpm)
{
    // Phoenix native encoder units are CPR / 100 msec
    return (rpm * kFlywheelCPR) / (60.0 * 10.0);
}

double Shooter::NativeToFlywheelRPM(double native)
{
    // Phoenix native encoder units are CPR / 100 msec
    return (native * 60.0 * 10.0) / kFlywheelCPR;
}

bool Shooter::IsAtDesiredRPM()
{
    bool atDesiredSpeed;
    static bool previousAtDesiredSpeed = true;

    if (m_state == SHOOTERSPEED_LOWHUB)
    {
        atDesiredSpeed = (fabs(m_flywheelLowerHubTargetRPM - m_flywheelCurrentRPM) < m_toleranceRPM);
    }
    else
    {
        atDesiredSpeed = (fabs(m_flywheelUpperHubTargetRPM - m_flywheelCurrentRPM) < m_toleranceRPM);
    }

    if ((m_state != SHOOTERSPEED_STOP) and !atDesiredSpeed)
        spdlog::info("SH m_flywheelCurrentRPM {:.1f}", m_flywheelCurrentRPM);

    if (atDesiredSpeed != previousAtDesiredSpeed)
    {
        spdlog::info("SH RPM at Speed {}", (atDesiredSpeed) ? "TRUE" : "FALSE");
        previousAtDesiredSpeed = atDesiredSpeed;
    }

    return atDesiredSpeed;
}

void Shooter::SetShooterSpeed(int state)
{
    double flywheelRPM = 0.0;

    m_state = state;

    spdlog::info("SH Set Shooter Speed {}", state);

    m_flywheelLowerHubTargetRPM =
        frc::SmartDashboard::GetNumber("SH_FlywheelLowerHubTargetRPM", m_flywheelLowerHubTargetRPM);
    m_flywheelUpperHubTargetRPM =
        frc::SmartDashboard::GetNumber("SH_FlywheelUpperHubTargetRPM", m_flywheelUpperHubTargetRPM);

    m_toleranceRPM = frc::SmartDashboard::GetNumber("SH_ToleranceRPM", m_toleranceRPM);

    if (m_talonValidSH11 && m_ifShooterTest)
    {
        m_flywheelPidKf = frc::SmartDashboard::GetNumber("SH_FlywheelPidKf", m_flywheelPidKf);
        m_flywheelPidKp = frc::SmartDashboard::GetNumber("SH_FlywheelPidKp", m_flywheelPidKp);
        m_flywheelPidKi = frc::SmartDashboard::GetNumber("SH_FlywheelPidKi", m_flywheelPidKi);
        m_flywheelPidKd = frc::SmartDashboard::GetNumber("SH_FlywheelPidKd", m_flywheelPidKd);

        m_motorSH11.Config_kF(kSlotIndex, m_flywheelPidKf, 0);
        m_motorSH11.Config_kP(kSlotIndex, m_flywheelPidKp, 0);
        m_motorSH11.Config_kI(kSlotIndex, m_flywheelPidKi, 0);
        m_motorSH11.Config_kD(kSlotIndex, m_flywheelPidKd, 0);
        m_motorSH11.SelectProfileSlot(kSlotIndex, kPidIndex);

        spdlog::info(
            "Flywheel Pid kF {:.4f} kP {:.4f} kI {:.4f} kD {:.4f}",
            m_flywheelPidKf,
            m_flywheelPidKp,
            m_flywheelPidKi,
            m_flywheelPidKd);
    }

    // // Validate and set the requested position to move
    switch (state)
    {
        case SHOOTERSPEED_REVERSE:
            flywheelRPM = -1000;
            break;
        case SHOOTERSPEED_STOP:
            flywheelRPM = 0.0;
            break;
        case SHOOTERSPEED_LOWHUB:
            flywheelRPM = m_flywheelLowerHubTargetRPM;
            break;
        case SHOOTERSPEED_HIGHHUB:
            flywheelRPM = m_flywheelUpperHubTargetRPM;
            break;
        case SHOOTERSPEED_PRIME:
            flywheelRPM = 200;
            break;
        default:
            spdlog::warn("SH invalid velocity requested - {}", state);
            break;
    }

    // Get current position in inches and set position mode and target counts

    if (m_talonValidSH11)
    {
        m_motorSH11.Set(ControlMode::Velocity, FlywheelRPMToNative(flywheelRPM));
    }

    spdlog::info("SH Set shooter speed -  flywheel {:.1f}", flywheelRPM);
}

void Shooter::ShooterReverseInit()
{
    SetShooterSpeed(SHOOTERSPEED_STOP);
}

void Shooter::ShooterReverseExecute()
{
    double reverseRPMThreshold = 20;

    if (m_flywheelCurrentRPM < reverseRPMThreshold)
    {
        m_motorSH11.ConfigPeakOutputReverse(-1.0);
        spdlog::info("Shooter can now reverse");
        SetShooterSpeed(SHOOTERSPEED_REVERSE);
    }
}

void Shooter::ShooterReverseEnd()
{
    m_motorSH11.ConfigPeakOutputReverse(0.0);
    SetShooterSpeed(SHOOTERSPEED_STOP);
}