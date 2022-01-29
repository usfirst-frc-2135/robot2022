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

#include <spdlog/spdlog.h>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

#include "commands/DrivetrainMotorTesting.h"

DrivetrainMotorTesting::DrivetrainMotorTesting(bool left, Drivetrain *m_drivetrain) :
    m_left(left),
    m_drivetrain(m_drivetrain)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("DrivetrainMotorTesting");
    AddRequirements(m_drivetrain);
}

// Called just before this Command runs the first time
void DrivetrainMotorTesting::Initialize()
{
    spdlog::info("DrivetrainMotorTesting - Init");
}

// Called repeatedly when this Command is scheduled to run
void DrivetrainMotorTesting::Execute()
{
    spdlog::info("DrivetrainMotorTesting - left {}", m_left);
    RobotContainer *robotContainer = RobotContainer::GetInstance();
    if (m_left)
        robotContainer->m_drivetrain.TankDriveVolts(3 * 1_V, 0 * 1_V);
    else
        robotContainer->m_drivetrain.TankDriveVolts(0 * 1_V, 3 * 1_V);
}

// Make this return true when this Command no longer needs to run execute()
bool DrivetrainMotorTesting::IsFinished()
{
    return false;
}

// Called once after isFinished returns true
void DrivetrainMotorTesting::End(bool interrupted)
{
    spdlog::info("DrivetrainMotorTesting - End");
}

bool DrivetrainMotorTesting::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}