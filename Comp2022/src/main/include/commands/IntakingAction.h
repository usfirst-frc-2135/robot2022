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

#include "commands/FloorConveyorRun.h"
#include "commands/IntakeDeploy.h"
#include "commands/IntakeRun.h"
#include "commands/VerticalConveyorRun.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitUntilCommand.h>

/**
 *
 *
 * @author ExampleAuthor
 */
class IntakingAction : public frc2::CommandHelper<frc2::ParallelCommandGroup, IntakingAction>
{
public:
    explicit IntakingAction(Intake *intake, FloorConveyor *fConv, VerticalConveyor *vConv);

    bool RunsWhenDisabled() const override;

private:
};
