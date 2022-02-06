// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "commands/AutoDriveShoot.h"

#include "commands/AutoDrivePath.h"
#include "commands/AutoDriveStop.h"
#include "commands/AutoDriveWait.h"
#include "commands/AutoPathSequence.h"
#include "commands/IntakeDeploy.h"
#include "commands/ScoringAction.h"
#include "frc2135/RobotConfig.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <spdlog/spdlog.h>
#include <wpi/SmallString.h>

AutoDriveShoot::AutoDriveShoot(
    Drivetrain *drivetrain,
    Intake *intake,
    FloorConveyor *fConv,
    VerticalConveyor *vConv,
    Shooter *shooter)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("AutoDriveShoot");

    // Add your commands here, e.g.
    // AddCommands(FooCommand(), BarCommand());
    frc2135::RobotConfig *config = frc2135::RobotConfig::GetInstance();
    config->GetValueAsString("AutoDriveShoot_path", m_pathname, "driveForward70");
    spdlog::info("AutoDriveShoot pathname {}", m_pathname.c_str());

    AddCommands(
        IntakeDeploy(true),
        AutoDriveWait(drivetrain),
        AutoDrivePath(m_pathname.c_str(), true, drivetrain),
        frc2::ParallelCommandGroup{ AutoDriveStop(drivetrain), ScoringAction(intake, fConv, vConv, shooter) });
}

bool AutoDriveShoot::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}