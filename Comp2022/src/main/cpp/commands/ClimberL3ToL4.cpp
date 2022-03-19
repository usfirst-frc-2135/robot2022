// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "commands/Climber3RotateToL3.h"
#include "commands/Climber5RotateIntoL3.h"
#include "commands/Climber7ClimbToL4.h"
#include "commands/ClimberMoveHeight.h"
#include "commands/ClimberSetGateHook.h"
#include "frc2135/RobotConfig.h"

#include <spdlog/spdlog.h>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

#include "commands/ClimberL3ToL4.h"

ClimberL3ToL4::ClimberL3ToL4(Climber *climber)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("ClimberL3ToL4");

    frc2135::RobotConfig *config = frc2135::RobotConfig::GetInstance();
    config->GetValueAsDouble("CL_RotateExtendL3Timer", m_rotateExtendL3Timer, 0.5);
    config->GetValueAsDouble("CL_RotateRetractL4Timer", m_rotateRetractL4Timer, 2.5);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    // Add your commands here, e.g.
    // AddCommands(FooCommand(), BarCommand());

    // state 4
    // Climber raises to 31.5 inches
    // Gate hook activated (extended) still

    // state 5
    // Climber stays at 31.5 inches (no movement)
    // Gate hook move to default position (closed)

    // state 7
    // Climber lowers to 25.25 inches
    // Gate hook is at default position

    AddCommands( // Sequential command
        ClimberRotateToL3(climber),
        frc2::WaitCommand(m_rotateExtendL3Timer * 1.0_s),
        ClimberRotateIntoL3(climber),
        frc2::WaitCommand(m_rotateRetractL4Timer * 1.0_s),
        ClimberClimbToL4(climber));
}

bool ClimberL3ToL4::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}