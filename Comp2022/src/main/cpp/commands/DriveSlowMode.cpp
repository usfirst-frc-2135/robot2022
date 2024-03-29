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

#include "commands/DriveSlowMode.h"

DriveSlowMode::DriveSlowMode(bool driveSlow) : m_driveSlow(driveSlow)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("DriveSlowMode");

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
}

// Called just before this Command runs the first time
void DriveSlowMode::Initialize()
{
    spdlog::info("DriveSlowMode - Init");
    RobotContainer *robotContainer = RobotContainer::GetInstance();
    robotContainer->m_drivetrain.SetDriveSlowMode(m_driveSlow);
}

// Called repeatedly when this Command is scheduled to run
void DriveSlowMode::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool DriveSlowMode::IsFinished()
{
    return true;
}

// Called once after isFinished returns true
void DriveSlowMode::End(bool interrupted)
{
    spdlog::info("DriveSlowMode - End");
}

bool DriveSlowMode::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}
