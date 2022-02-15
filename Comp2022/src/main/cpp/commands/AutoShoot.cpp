// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "commands/AutoShoot.h"

#include "commands/AutoStop.h"
#include "commands/AutoWait.h"
#include "commands/IntakeDeploy.h"
#include "commands/ScoringAction.h"
#include "commands/ShooterRunTimeout.h"
#include "frc2135/RobotConfig.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <spdlog/spdlog.h>
#include <wpi/SmallString.h>

AutoShoot::AutoShoot(
    Drivetrain *drivetrain,
    Intake *intake,
    FloorConveyor *fConv,
    VerticalConveyor *vConv,
    Shooter *shooter)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("AutoShoot");

    // Add your commands here, e.g.
    // AddCommands(FooCommand(), BarCommand());
    AddCommands(
        frc2::ParallelRaceGroup{ IntakeDeploy(true), AutoStop(drivetrain) },
        AutoWait(drivetrain),
        frc2::ParallelRaceGroup{ ShooterRunTimeout(Shooter::SHOOTERSPEED_FORWARD, shooter), AutoStop(drivetrain) },
        frc2::ParallelRaceGroup{ ScoringAction(intake, fConv, vConv, shooter), AutoStop(drivetrain) });
}

bool AutoShoot::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}