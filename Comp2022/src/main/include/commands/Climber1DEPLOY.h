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
#include "commands/ClimberMoveHeight.h"
#include "commands/ClimberSetGateHook.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

#include "subsystems/Climber.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include <frc2/command/SequentialCommandGroup.h>

using namespace std;

class ClimberDeploy : public frc2::CommandHelper<frc2::SequentialCommandGroup, ClimberDeploy>
{
public:
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    explicit ClimberDeploy(Climber *m_climber);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

    bool RunsWhenDisabled() const override;

private:
};
