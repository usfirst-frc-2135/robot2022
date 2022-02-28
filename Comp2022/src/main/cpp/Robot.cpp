// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "Robot.h"

#include "frc2135/RobotConfig.h"
#include "frc2135/spdlog.h"

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <spdlog/spdlog.h>

std::string MatchTypeToString(const frc::DriverStation::MatchType matchType)
{
    switch (matchType)
    {
        case frc::DriverStation::MatchType::kNone:
            return "N";
        case frc::DriverStation::MatchType::kPractice:
            return "P";
        case frc::DriverStation::MatchType::kQualification:
            return "Q";
        case frc::DriverStation::MatchType::kElimination:
            return "E";
    }
    return "<unknown>";
}

std::string AllianceToString(const frc::DriverStation::Alliance alliance)
{
    switch (alliance)
    {
        case frc::DriverStation::Alliance::kRed:
            return "Red";
        case frc::DriverStation::Alliance::kBlue:
            return "Blue";
        case frc::DriverStation::Alliance::kInvalid:
            return "Invalid";
    }
    return "<unknown>";
}

void Robot::RobotInit()
{
    std::string robotName;
    frc2135::RobotConfig *config = frc2135::RobotConfig::GetInstance();

    spdlog::info("RB Init");
    config->GetValueAsString("RB_Name", robotName, "unknown");
    spdlog::info("RobotInit: name - {}", robotName);

    // Retrieve auto values from config file and put on smartdashboard
    double waitTime;
    config->GetValueAsDouble("AUTO_WaitTime", waitTime, 0.0);
    frc::SmartDashboard::PutNumber("AUTO_WaitTime", waitTime);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit()
{
    spdlog::info(
        "DisabledInit: Match {}{}, {} Alliance",
        MatchTypeToString(frc::DriverStation::GetMatchType()),
        frc::DriverStation::GetMatchNumber(),
        AllianceToString(frc::DriverStation::GetAlliance()));
    RobotContainer *robotContainer = RobotContainer::GetInstance();
    robotContainer->m_drivetrain.Initialize();
    robotContainer->m_intake.Initialize();
    robotContainer->m_floorConv.Initialize();
    robotContainer->m_vertConv.Initialize();
    robotContainer->m_shooter.Initialize();
    robotContainer->m_climber.Initialize();
    robotContainer->m_pneumatics.Initialize();
    robotContainer->m_power.Initialize();
    robotContainer->m_led.Initialize();
    robotContainer->m_vision.Initialize();
}

void Robot::DisabledPeriodic()
{
    // If RoboRIO User button is pressed, dump all CAN faults
    if (frc::RobotController::GetUserButton())
    {
        if (!m_faultsCleared)
        {
            m_faultsCleared = true;
            RobotFaultDump();
        }
    }
    else if (m_faultsCleared)
    {
        m_faultsCleared = false;
    }
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit()
{
    spdlog::info(
        "AutonomousInit: Match {}{}, {} Alliance",
        MatchTypeToString(frc::DriverStation::GetMatchType()),
        frc::DriverStation::GetMatchNumber(),
        AllianceToString(frc::DriverStation::GetAlliance()));

    m_autonomousCommand = m_container->GetAutonomousCommand();

    if (m_autonomousCommand != nullptr)
    {
        m_autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
    spdlog::info(
        "TeleopInit: Match {}{}, {} Alliance",
        MatchTypeToString(frc::DriverStation::GetMatchType()),
        frc::DriverStation::GetMatchNumber(),
        AllianceToString(frc::DriverStation::GetAlliance()));

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != nullptr)
    {
        m_autonomousCommand->Cancel();
        m_autonomousCommand = nullptr;
    }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

///////////////////////////////////////////////////////////////////////////////////////////////////
//  Fault handling utilities

void Robot::RobotFaultDump(void)
{
    // Print out talon SRX faults and clear sticky ones
    spdlog::info("----- DUMP FAULTS --------------");

    RobotContainer *robotContainer = RobotContainer::GetInstance();
    robotContainer->m_drivetrain.FaultDump();
    robotContainer->m_intake.FaultDump();
    robotContainer->m_floorConv.FaultDump();
    robotContainer->m_vertConv.FaultDump();
    robotContainer->m_shooter.FaultDump();
    robotContainer->m_climber.FaultDump();
    robotContainer->m_pneumatics.FaultDump();
    robotContainer->m_power.FaultDump();
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    frc2135::initialize_spdlog();
    return frc::StartRobot<Robot>();
}
#endif
