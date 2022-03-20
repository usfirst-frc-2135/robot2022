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

#include "commands/ShooterAimToggle.h"

ShooterAimToggle::ShooterAimToggle(void)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("ShooterAimToggle");

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
}

// Called just before this Command runs the first time
void ShooterAimToggle::Initialize()
{
    spdlog::info("ShooterAimToggle - Init");
    RobotContainer *robotContainer = RobotContainer::GetInstance();

    if (robotContainer->m_vision.GetLEDMode() == Vision::LED_ON)
    {
        robotContainer->m_vision.SetLEDMode(Vision::LED_OFF);
        robotContainer->m_vision.SetCameraDisplay(Vision::PIP_SECONDARY);
    }
    else
    {
        robotContainer->m_vision.SetLEDMode(Vision::LED_ON);
        robotContainer->m_vision.SetCameraDisplay(Vision::PIP_MAIN);
    }
}

// Called repeatedly when this Command is scheduled to run
void ShooterAimToggle::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool ShooterAimToggle::IsFinished()
{
    return true;
}

// Called once after isFinished returns true
void ShooterAimToggle::End(bool interrupted)
{
    spdlog::info("ShooterAimToggle - End");
}

bool ShooterAimToggle::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return true;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}