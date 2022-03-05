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
#include <frc/RobotState.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "subsystems/Climber.h"

#include <frc/smartdashboard/SmartDashboard.h>

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

Climber::Climber()
{
    SetName("Climber");
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    SetSubsystem("Climber");

    AddChild("Brake", &m_gatehook);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // Validate Talon SRX controllers, initialize and display firmware versions
    m_talonValidCL14 = frc2135::TalonUtils::TalonCheck(m_motorCL14, "CL", "CL14");
    m_talonValidCL15 = frc2135::TalonUtils::TalonCheck(m_motorCL15, "CL", "CL15");

    // Check if solenoids are functional or blacklisted
    if (m_gatehook.IsDisabled())
        spdlog::error("CL Climber Solenoid is BLACKLISTED");
    else
        spdlog::info("CL Climber Solenoid is FUNCTIONAL");

    // Initialize Variables
    frc2135::RobotConfig *config = frc2135::RobotConfig::GetInstance();
    config->GetValueAsDouble("CL_PeakOut", m_peakOut, 1.0);
    config->GetValueAsInt("CL_Velocity", m_velocity, 21776);
    config->GetValueAsInt("CL_Acceleration", m_acceleration, 43552);
    config->GetValueAsInt("CL_SCurveStrength", m_sCurveStrength, 0);
    config->GetValueAsDouble("CL_PidKf", m_pidKf, 0.0);
    config->GetValueAsDouble("CL_PidKp", m_pidKp, 0.0);
    config->GetValueAsDouble("CL_PidKi", m_pidKi, 0.000);
    config->GetValueAsDouble("CL_PidKd", m_pidKd, 0.000);
    config->GetValueAsDouble("CL_CLRampRate", m_CLRampRate, 0.000);
    config->GetValueAsInt("CL_CLAllowedError", m_CLAllowedError, 0);
    config->GetValueAsDouble("CL_ToleranceInches", m_toleranceInches, 0.75);
    config->GetValueAsDouble("CL_MaxHeight", m_climberMaxHeight, 85.0);
    config->GetValueAsDouble("CL_MinHeight", m_climberMinHeight, 0.0);
    config->GetValueAsDouble("CL_StowHeight", m_stowHeight, 0.25);
    config->GetValueAsDouble("CL_ExtendL2", m_extendL2, 29.0);
    config->GetValueAsDouble("CL_RotateL3", m_rotateL3, 21.0);
    config->GetValueAsDouble("CL_ExtendL3", m_extendL3, 31.5);
    config->GetValueAsDouble("CL_GatehookRestHeight", m_gatehookRestHeight, 0.35);
    config->GetValueAsDouble("CL_RaiseL4", m_raiseL4, 25.25);

    frc::SmartDashboard::PutNumber("CL_PidKf", m_pidKf);
    frc::SmartDashboard::PutNumber("CL_Velocity", m_velocity);
    frc::SmartDashboard::PutNumber("CL_Acceleration", m_acceleration);
    frc::SmartDashboard::PutNumber("CL_SCurveStrength", m_sCurveStrength);
    frc::SmartDashboard::PutNumber("CL_PidKp", m_pidKp);
    frc::SmartDashboard::PutNumber("CL_PidKi", m_pidKi);
    frc::SmartDashboard::PutNumber("CL_PidKd", m_pidKd);
    frc::SmartDashboard::PutNumber("CL_StowHeight", m_stowHeight);
    frc::SmartDashboard::PutNumber("CL_ExtendL2", m_extendL2);
    frc::SmartDashboard::PutNumber("CL_RotateL3", m_rotateL3);
    frc::SmartDashboard::PutNumber("CL_ExtendL3", m_extendL3);
    frc::SmartDashboard::PutNumber("CL_GatehookRestHeight", m_gatehookRestHeight);
    frc::SmartDashboard::PutNumber("CL_RaiseL4", m_raiseL4);

    // Magic Motion variables
    m_curInches = 0.0;
    m_targetInches = 0.0;
    m_calibrated = false;

    // Field for manually progamming climber height
    frc::SmartDashboard::PutBoolean("CL_Calibrated", m_calibrated);

    // Set motor directions
    // Turn on Coast mode (not brake)
    // Set motor peak outputs
    if (m_talonValidCL14)
    {
        m_motorCL14.SetInverted(true);
        m_motorCL14.SetNeutralMode(NeutralMode::Brake);
        m_motorCL14.SetSafetyEnabled(false);

        m_motorCL14.ConfigSupplyCurrentLimit(m_supplyCurrentLimits);
        m_motorCL14.ConfigStatorCurrentLimit(m_statorCurrentLimits);

        // Configure sensor settings
        m_motorCL14.SetSelectedSensorPosition(0, 0, kCANTimeout);

        // Set allowable closed loop error
        m_motorCL14.ConfigAllowableClosedloopError(0, m_CLAllowedError, kCANTimeout);

        // Set soft limits for climber height
        //m_motorCL14.ConfigForwardSoftLimitThreshold(InchesToCounts(m_climberMaxHeight), kCANTimeout);
        //m_motorCL14.ConfigReverseSoftLimitThreshold(InchesToCounts(m_climberMinHeight), kCANTimeout);
        // m_motorCL14.ConfigForwardSoftLimitEnable(true, kCANTimeout);
        // m_motorCL14.ConfigReverseSoftLimitEnable(true, kCANTimeout);

        // Configure Magic Motion settings
        m_motorCL14.SelectProfileSlot(0, 0);
        m_motorCL14.ConfigMotionCruiseVelocity(m_velocity, kCANTimeout);
        m_motorCL14.ConfigMotionAcceleration(m_acceleration, kCANTimeout);
        m_motorCL14.ConfigMotionSCurveStrength(m_sCurveStrength, kCANTimeout);
        m_motorCL14.Config_kF(0, m_pidKf, kCANTimeout);
        m_motorCL14.Config_kP(0, m_pidKp, kCANTimeout);
        m_motorCL14.Config_kI(0, m_pidKi, kCANTimeout);
        m_motorCL14.Config_kD(0, m_pidKd, kCANTimeout);

        m_motorCL14.Set(ControlMode::PercentOutput, 0.0);
    }

    if (m_talonValidCL15)
    {
        m_motorCL15.Set(ControlMode::Follower, 14);
        m_motorCL15.SetInverted(InvertType::OpposeMaster);
        m_motorCL15.SetNeutralMode(NeutralMode::Brake);
        m_motorCL15.ConfigSupplyCurrentLimit(m_supplyCurrentLimits);
        m_motorCL15.ConfigStatorCurrentLimit(m_statorCurrentLimits);
        m_motorCL15.SetStatusFramePeriod(Status_1_General_, 255, kCANTimeout);
        m_motorCL15.SetStatusFramePeriod(Status_2_Feedback0_, 255, kCANTimeout);
    }

    Initialize();
}

void Climber::Periodic()
{
    // Put code here to be run every loop
    static int periodicInterval = 0;
    double outputCL14 = 0.0;

    // if disabled
    if (frc::RobotState::IsDisabled())
    {
        RobotContainer *robotContainer = RobotContainer::GetInstance();
        if (!m_climberDownLeft.Get() || !m_climberDownRight.Get())
        {
            robotContainer->m_led.SetColor(LED::LEDCOLOR_BLUE);
        }
        else
        {
            robotContainer->m_led.SetColor(LED::LEDCOLOR_OFF);
        }
    }

    if (m_talonValidCL14)
        outputCL14 = m_motorCL14.GetMotorOutputPercent();
    frc::SmartDashboard::PutNumber("CL_Output_CL14", outputCL14);

    int curCounts = 0;
    if (m_talonValidCL14)
    {
        curCounts = m_motorCL14.GetSelectedSensorPosition(0);
    }

    m_curInches = CountsToInches(curCounts);
    frc::SmartDashboard::PutNumber("CL_Height", m_curInches);

    // Only update indicators every 100 ms to cut down on network traffic
    if (periodicInterval++ % 5 == 0)
    {
        if (m_climberDebug > 0)
        {
            double currentCL14 = 0.0;
            double currentCL15 = 0.0;

            if (m_talonValidCL14)
                currentCL14 = m_motorCL14.GetOutputCurrent();
            if (m_talonValidCL15)
                currentCL15 = m_motorCL15.GetOutputCurrent();

            frc::SmartDashboard::PutNumber("CL_Current_CL14", currentCL14);
            frc::SmartDashboard::PutNumber("CL_Current_CL15", currentCL15);
        }
    }
}

void Climber::SimulationPeriodic()
{
    // This method will be called once per scheduler run when in simulation
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

// Put methods for controlling this subsystem
// here. Call these from Commands.

void Climber::Initialize(void)
{
    double curCounts = 0.0;

    spdlog::info("CL Init");

    SetGateHook(false);

    SetClimberStopped();

    if (m_talonValidCL14)
        curCounts = m_motorCL14.GetSelectedSensorPosition(0);

    m_curInches = CountsToInches(curCounts);
    m_targetInches = m_curInches;
    m_isMoving = false;
    spdlog::info("CL Init Target Inches: {}", m_targetInches);
}

// Dump all Talon faults
void Climber::FaultDump(void)
{
    frc2135::TalonUtils::TalonFaultDump("CL 14", m_motorCL14);
}

// Raise climber with Joysticks method
void Climber::MoveClimberWithJoysticks(frc::XboxController *joystick)
{
    static int state = CLIMBER_INIT;
    double yCLValue = 0.0;
    double motorOutput = 0.0;

    yCLValue = -joystick->GetLeftY();

    if (yCLValue > -0.1 && yCLValue < 0.1)
    {
        if (state != CLIMBER_STOPPED)
            spdlog::info("CL Climber Stopped");
        state = CLIMBER_STOPPED;
    }
    else
    {
        // If joystick is above a value, climber will move up
        if (yCLValue > m_deadband)
        {
            if (state != CLIMBER_DOWN)
                spdlog::info("CL Climber Down");
            state = CLIMBER_DOWN;

            yCLValue -= m_deadband;
            yCLValue *= (1.0 / (1.0 - m_deadband));
            motorOutput = (0.3) * (yCLValue * abs(yCLValue));
        }
        // If joystick is below a value, climber will move down
        else if (yCLValue < -m_deadband)
        {
            if (state != CLIMBER_UP)
                spdlog::info("CL Climber Up");
            state = CLIMBER_UP;

            yCLValue += m_deadband;
            yCLValue *= (1.0 / (1.0 - m_deadband));
            motorOutput = (0.3) * (yCLValue * abs(yCLValue));
        }
    }

    if (m_talonValidCL14)
        m_motorCL14.Set(ControlMode::PercentOutput, motorOutput);
}

void Climber::SetClimberStopped(void)
{
    spdlog::info("CL Set Climber Stopped");

    if (m_talonValidCL14)
        m_motorCL14.Set(ControlMode::PercentOutput, 0);
}

void Climber::SetGateHook(bool hookClosed)
{
    if (hookClosed != m_gatehook.Get())
    {
        spdlog::info("CL HOOK {}", (hookClosed) ? "OPEN" : "CLOSED");
        frc::SmartDashboard::PutBoolean("CL_Hook_Closed", hookClosed);

        m_gatehook.Set(hookClosed);
    }
}

///////////////////////// MOTION MAGIC ///////////////////////////////////

double Climber::InchesToCounts(double inches)
{
    return inches / kInchesPerCount;
}

double Climber::CountsToInches(int counts)
{
    return counts * kInchesPerCount;
}

void Climber::Calibrate()
{
    if (m_talonValidCL14)
    {
        m_motorCL14.SetSelectedSensorPosition(0, 0, kCANTimeout);
        m_motorCL14.Set(ControlMode::Position, 0.0);
    }
    m_calibrated = true;
    frc::SmartDashboard::PutBoolean("CL_Calibrated", m_calibrated);
}

void Climber::MoveClimberDistanceInit(int state)
{
    m_pidKf = frc::SmartDashboard::GetNumber("CL_PidKf", m_pidKf);
    m_velocity = frc::SmartDashboard::GetNumber("CL_Velocity", m_velocity);
    m_acceleration = frc::SmartDashboard::GetNumber("CL_Acceleration", m_acceleration);
    m_sCurveStrength = frc::SmartDashboard::GetNumber("CL_SCurveStrength", m_sCurveStrength);
    m_pidKp = frc::SmartDashboard::GetNumber("CL_PidKp", m_pidKp);
    m_pidKi = frc::SmartDashboard::GetNumber("CL_PidKi", m_pidKi);
    m_pidKd = frc::SmartDashboard::GetNumber("CL_PidKd", m_pidKd);

    m_motorCL14.Config_kF(0, m_pidKf, 0);
    m_motorCL14.ConfigMotionCruiseVelocity(m_velocity, 0);
    m_motorCL14.ConfigMotionAcceleration(m_acceleration, 0);
    m_motorCL14.ConfigMotionSCurveStrength(m_sCurveStrength, 0);
    m_motorCL14.Config_kP(0, m_pidKp, 0);
    m_motorCL14.Config_kI(0, m_pidKi, 0);
    m_motorCL14.Config_kD(0, m_pidKd, 0);

    switch (state)
    {
        case NOCHANGE_HEIGHT: // Do not change from current level!
            break;
        case STOW_HEIGHT:
            m_targetInches = frc::SmartDashboard::GetNumber("CL_StowHeight", m_stowHeight);
            break;
        case EXTEND_L2_HEIGHT:
            m_targetInches = frc::SmartDashboard::GetNumber("CL_ExtendL2", m_extendL2);
            break;
        case ROTATE_L3_HEIGHT:
            m_targetInches = frc::SmartDashboard::GetNumber("CL_RotateL3", m_rotateL3);
            break;
        case EXTEND_L3_HEIGHT:
            m_targetInches = frc::SmartDashboard::GetNumber("CL_ExtendL3", m_rotateL3);
            break;
        case GATEHOOK_REST_HEIGHT:
            m_targetInches = frc::SmartDashboard::GetNumber("CL_GatehookRestHeight", m_gatehookRestHeight);
            break;
        case RAISE_L4_HEIGHT:
            m_targetInches = frc::SmartDashboard::GetNumber("CL_RaiseL4", m_raiseL4);
            break;
        default:
            spdlog::info("CL requested height is invalid - {}", state);
            return;
    }

    if (m_calibrated)
    {
        // Height constraint check/soft limit for max and min height before raising
        if (m_targetInches < m_climberMinHeight)
        {
            spdlog::info("Target {} inches is limited by {} inches", m_targetInches, m_climberMinHeight);
            m_targetInches = m_climberMinHeight;
        }

        if (m_targetInches > m_climberMaxHeight)
        {
            spdlog::info("Target {} inches is limited by {} inches", m_targetInches, m_climberMaxHeight);
            m_targetInches = m_climberMaxHeight;
        }

        // Start the safety timer
        m_safetyTimeout = 4.0_s;
        m_safetyTimer.Reset();
        m_safetyTimer.Start();

        m_motorCL14.Set(ControlMode::MotionMagic, InchesToCounts(m_targetInches));

        spdlog::info(
            "Climber moving: {} -> {} inches  |  counts {:.0f} -> {:.0f}",
            m_curInches,
            m_targetInches,
            InchesToCounts(m_curInches),
            InchesToCounts(m_targetInches));
    }
    else
    {
        spdlog::info("Climber is not calibrated");
        if (m_talonValidCL14)
            m_motorCL14.Set(ControlMode::PercentOutput, 0.0);
    }
}

bool Climber::MoveClimberDistanceIsFinished()
{
    static int withinTolerance = 0;
    bool isFinished = false;
    double errorInInches = 0.0;

    errorInInches = m_targetInches - m_curInches;

    // Check to see if the error is in an acceptable number of inches
    if (fabs(errorInInches) < m_toleranceInches)
    {
        if (++withinTolerance >= 5)
        {
            isFinished = true;
            spdlog::info(
                "Climber move finished - Time: {:.5f}  |  Cur inches: {:.4f}",
                m_safetyTimer.Get(),
                m_curInches);
        }
    }
    else
    {
        withinTolerance = 0;
    }

    // Check to see if the Safety Timer has timed out
    if (m_safetyTimer.Get() >= m_safetyTimeout)
    {
        isFinished = true;
        spdlog::info("Climber Move Safety timer has timed out");
    }

    // If completed, clean up
    if (isFinished)
    {
        withinTolerance = 0;
        m_safetyTimer.Stop();
        m_isMoving = false;
    }

    return isFinished;
}
