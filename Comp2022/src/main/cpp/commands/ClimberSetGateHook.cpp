// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include <RobotContainer.h>
#include <spdlog/spdlog.h>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

#include "commands/ClimberSetGateHook.h"

ClimberSetGateHook::ClimberSetGateHook(bool climberSetGateHook) : m_climberGateHookClosed(climberSetGateHook)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("ClimberSetGateHook");

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
}

// Called just before this Command runs the first time
void ClimberSetGateHook::Initialize()
{
    spdlog::info("ClimberSetGateHook - Init");
    RobotContainer *robotContainer = RobotContainer::GetInstance();
    robotContainer->m_climber.SetGateHook(m_climberGateHookClosed);
}

// Called repeatedly when this Command is scheduled to run
void ClimberSetGateHook::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool ClimberSetGateHook::IsFinished()
{
    return true;
}

// Called once after isFinished returns true
void ClimberSetGateHook::End(bool interrupted)
{
    spdlog::info("ClimberSetGateHook - End");
}

bool ClimberSetGateHook::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}
