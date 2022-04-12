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

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

/**
 * https://docs.google.com/presentation/d/1SQlcOZpuv9mqVmsuJRcJQMyKcwuBd6hcQGlkFT-_Odg/edit?pli=1#slide=id.g11030b7b884_1_28
 *
 * @author ngupta-22
 */
class Auto1Ball2OppLeft : public frc2::CommandHelper<frc2::SequentialCommandGroup, Auto1Ball2OppLeft>
{
public:
    explicit Auto1Ball2OppLeft(
        Drivetrain *m_drivetrain,
        Intake *m_intake,
        FloorConveyor *m_floorConv,
        VerticalConveyor *m_vertConv,
        Shooter *m_shooter);

    bool RunsWhenDisabled() const override;

private:
    // Must be a member variable so commands can use it when they execute
    std::string m_pathname1;
    std::string m_pathname2;
    std::string m_pathname3;
    std::string m_pathname4;
    std::string m_pathname5;
};
