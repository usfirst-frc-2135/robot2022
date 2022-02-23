// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "commands/Climber1Deploy.h"
#include "commands/Climber2ClimbToL2.h"
#include "commands/Climber3RotateToL3.h"
#include "commands/Climber5RotateIntoL3.h"
#include "commands/Climber6ClimbToL3.h"
#include "commands/Climber7ClimbToL4.h"
#include "commands/Climber8SettleToL4.h"
#include "frc2135/RobotConfig.h"

#include <spdlog/spdlog.h>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

#include "commands/ClimberFullClimb.h"

ClimberFullClimb::ClimberFullClimb(Climber *climber)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("ClimberFullClimb");

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    // Add your commands here, e.g.
    // AddCommands(FooCommand(), BarCommand());

    // state 2
    // Climber lower to 0.25 inches
    // Gate hook is at default position (closed)
    // Climber raise to 0.35 inches (so hooks can rest)
    // Wait 0.5 second

    // state 3
    // Climber raises to 21 inches
    // Gate hook activated (extended)

    // state 4
    // Climber raises to 31.5 inches
    // Gate hook activated (extended) still

    // state 5
    // Climber stays at 31.5 inches (no movement)
    // Gate hook move to default position (closed)

    // state 6
    // Climber lowers to 0.35 inches
    // Gate hook is at default position

    // REPEAT STATES 2 - 5

    // state 7
    // Climber lowers to 25.25 inches
    // Gate hook is at default position

    // state 8
    // Climber lowers to 0.35 inches
    // Gate hook is at default position

    AddCommands(
        ClimberClimbToL2(climber),
        ClimberRotateToL3(climber),
        frc2::WaitCommand(0.5_s),
        ClimberRotateIntoL3(climber),
        frc2::WaitCommand(0.2_s),
        ClimberClimbToL3(climber),
        // next rung climb!
        ClimberRotateToL3(climber),
        frc2::WaitCommand(0.5_s),
        ClimberRotateIntoL3(climber),
        frc2::WaitCommand(0.2_s),
        ClimberClimbToL4(climber),
        ClimberSettleToL4(climber));
}

bool ClimberFullClimb::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}