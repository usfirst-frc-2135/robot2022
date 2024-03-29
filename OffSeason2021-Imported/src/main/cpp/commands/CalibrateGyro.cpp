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

#include "commands/CalibrateGyro.h"

CalibrateGyro::CalibrateGyro(Drivetrain *m_drivetrain) : m_drivetrain(m_drivetrain)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("CalibrateGyro");
    AddRequirements(m_drivetrain);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
}

// Called just before this Command runs the first time
void CalibrateGyro::Initialize()
{
    spdlog::info("CalibrateGyro - Init");
    m_drivetrain->CalibrateGyro();
    m_drivetrain->ResetSensors();
}

// Called repeatedly when this Command is scheduled to run
void CalibrateGyro::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool CalibrateGyro::IsFinished()
{
    return true;
}

// Called once after isFinished returns true
void CalibrateGyro::End(bool interrupted)
{
    spdlog::info("CalibrateGyro - End");
}

bool CalibrateGyro::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return true;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}
