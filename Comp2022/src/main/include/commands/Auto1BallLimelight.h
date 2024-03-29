// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#pragma once

#include "subsystems/Drivetrain.h"
#include "subsystems/FloorConveyor.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"
#include "subsystems/VerticalConveyor.h"
#include "subsystems/Vision.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

/**
 *      
 *
 * @author ngupta-22
 */
class Auto1BallLimelight : public frc2::CommandHelper<frc2::SequentialCommandGroup, Auto1BallLimelight>
{
public:
    explicit Auto1BallLimelight(
        Drivetrain *m_drivetrain,
        Intake *m_intake,
        FloorConveyor *m_floorConv,
        VerticalConveyor *m_vertConv,
        Shooter *m_shooter,
        Vision *m_vision);

    bool RunsWhenDisabled() const override;

private:
    // Must be a member variable so commands can use it when they execute
    std::string m_pathname1;
    std::string m_pathname2;
};
