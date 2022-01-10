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

#include "commands/ClimberRun.h"

ClimberRun::ClimberRun(Climber *m_climber) : m_climber(m_climber)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("ClimberRun");
    AddRequirements(m_climber);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
}

// Called just before this Command runs the first time
void ClimberRun::Initialize()
{
    spdlog::info("ClimberRun - Init");
}

// Called repeatedly when this Command is scheduled to run
void ClimberRun::Execute()
{
    m_climber->RaiseClimberWithJoysticks(RobotContainer::GetInstance()->getOperatorController());
}

// Make this return true when this Command no longer needs to run execute()
bool ClimberRun::IsFinished()
{
    return false;
}

// Called once after isFinished returns true
void ClimberRun::End(bool interrupted)
{
    spdlog::info("ClimberRun - End");
}

bool ClimberRun::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}
