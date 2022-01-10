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

#include "commands/DriveBackward.h"

DriveBackward::DriveBackward(Drivetrain *m_drivetrain, Vision *m_vision) :
    m_drivetrain(m_drivetrain),
    m_vision(m_vision)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("DriveBackward");
    AddRequirements(m_drivetrain);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
}

// Called just before this Command runs the first time
void DriveBackward::Initialize()
{
    spdlog::info("DriveBackward - Init");
    m_vision->SetLEDMode(Vision::LED_ON);
    m_drivetrain->MoveWithLimelightInit();
}

// Called repeatedly when this Command is scheduled to run
void DriveBackward::Execute()
{
    RobotContainer *robotContainer = RobotContainer::GetInstance();
    double tx = robotContainer->m_vision.GetHorizOffsetDeg();
    double ty = robotContainer->m_vision.GetVertOffsetDeg();
    bool tv = robotContainer->m_vision.GetTargetValid();
    m_drivetrain->DriveBackward(tx, ty, tv);
}

// Make this return true when this Command no longer needs to run execute()
bool DriveBackward::IsFinished()
{
    RobotContainer *robotContainer = RobotContainer::GetInstance();
    double tx = robotContainer->m_vision.GetHorizOffsetDeg();
    bool tv = robotContainer->m_vision.GetTargetValid();
    return m_drivetrain->MoveWithLimelightIsFinished(tx, tv);
}

// Called once after isFinished returns true
void DriveBackward::End(bool interrupted)
{
    spdlog::info("DriveBackward - End");
    m_vision->SetLEDMode(Vision::LED_OFF);
    m_drivetrain->MoveWithLimelightEnd();
}

bool DriveBackward::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}
