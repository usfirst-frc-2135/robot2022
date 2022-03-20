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

#include "commands/DriveLimelight.h"

DriveLimelight::DriveLimelight(bool endAtTarget, Drivetrain *m_drivetrain, Vision *m_vision) :
    m_endAtTarget(endAtTarget),
    m_drivetrain(m_drivetrain),
    m_vision(m_vision)

{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("DriveLimelight");
    AddRequirements(m_drivetrain);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
}

// Called just before this Command runs the first time
void DriveLimelight::Initialize()
{
    spdlog::info("DriveLimelight - Init HERE");
    m_vision->SetLEDMode(Vision::LED_ON);
    m_vision->SetCameraDisplay(Vision::PIP_MAIN);
    m_drivetrain->MoveWithLimelightInit(m_endAtTarget);
}

// Called repeatedly when this Command is scheduled to run
void DriveLimelight::Execute()
{
    m_drivetrain->MoveWithLimelightExecute();
}

// Make this return true when this Command no longer needs to run execute()
bool DriveLimelight::IsFinished()
{
    //spdlog::info("DTL m_endAtTarget {}", m_endAtTarget);
    if (m_endAtTarget)
        return m_drivetrain->MoveWithLimelightIsFinished();
    return false;
}

// Called once after isFinished returns true
void DriveLimelight::End(bool interrupted)
{
    spdlog::info("DriveLimelight - End");
    m_vision->SetLEDMode(Vision::LED_OFF);
    m_vision->SetCameraDisplay(Vision::PIP_SECONDARY);
    m_drivetrain->MoveWithLimelightEnd();
}

bool DriveLimelight::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}
