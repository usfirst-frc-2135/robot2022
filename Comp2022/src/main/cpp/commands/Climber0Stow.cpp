// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "frc2135/RobotConfig.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <spdlog/spdlog.h>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

#include "commands/Climber0Stow.h"

ClimberStow::ClimberStow(
    Climber *climber,
    Intake *intake,
    FloorConveyor *fConv,
    VerticalConveyor *vConv,
    Shooter *shooter)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("ClimberStow");

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    // Add your commands here, e.g.
    // AddCommands(FooCommand(), BarCommand());

    // Return subsystems to default state
    // Climber starts at 0.25 inches
    // Gate hook is at default position (closed)

    AddCommands( // Sequential command
        DriveSlowMode(false),
        IntakeDeploy(false),
        IntakeRun(Intake::INTAKE_STOP, intake),
        FloorConveyorRun(FloorConveyor::FCONVEYOR_STOP, fConv),
        VerticalConveyorRun(VerticalConveyor::VCONVEYOR_STOP, vConv),
        ShooterRun(Shooter::SHOOTERSPEED_STOP, shooter),
        frc2::ParallelDeadlineGroup{
            frc2::WaitUntilCommand([climber] { return climber->MoveClimberDistanceIsFinished(); }),
            ClimberMoveHeight(Climber::STOW_HEIGHT, climber) },
        ClimberSetGateHook(false));
}

bool ClimberStow::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}