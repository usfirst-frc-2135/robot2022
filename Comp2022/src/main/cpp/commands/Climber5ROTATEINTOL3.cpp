// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "commands/ClimberMoveHeight.h"
#include "commands/ClimberSetGateHook.h"
#include "frc2135/RobotConfig.h"

#include <spdlog/spdlog.h>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

#include "commands/Climber5RotateIntoL3.h"

ClimberRotateIntoL3::ClimberRotateIntoL3(Climber *climber)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("ClimberRotateIntoL3");

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    // Add your commands here, e.g.
    // AddCommands(FooCommand(), BarCommand());

    // Climber stays at 31.5 inches (no movement)
    // Gate hook move to default position (closed)

    AddCommands(
        ClimberMoveHeight(Climber::NOCHANGE_HEIGHT, climber),
        ClimberSetGateHook(false),
        frc2::WaitCommand(0.5_s));
}

bool ClimberRotateIntoL3::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}