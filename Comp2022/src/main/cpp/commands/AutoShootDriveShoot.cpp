// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "commands/AutoShootDriveShoot.h"

#include "commands/AutoDrivePath.h"
#include "commands/AutoPathSequence.h"
#include "commands/AutoStop.h"
#include "commands/AutoWait.h"
#include "commands/IntakeDeploy.h"
#include "commands/IntakingAction.h"
#include "commands/ScoringAction.h"
#include "commands/ScoringStop.h"
#include "frc2135/RobotConfig.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <spdlog/spdlog.h>
#include <wpi/SmallString.h>

AutoShootDriveShoot::AutoShootDriveShoot(
    Drivetrain *drivetrain,
    Intake *intake,
    FloorConveyor *fConv,
    VerticalConveyor *vConv,
    Shooter *shooter)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("AutoShootDriveShoot");

    // Add your commands here, e.g.
    // AddCommands(FooCommand(), BarCommand());
    frc2135::RobotConfig *config = frc2135::RobotConfig::GetInstance();
    config->GetValueAsString("AutoShootDriveShoot_path1", m_pathname1, "forward79");
    config->GetValueAsString("AutoShootDriveShoot_path2", m_pathname2, "forward79");
    spdlog::info("AutoShootDriveShoot pathname 1 {}", m_pathname1.c_str());
    spdlog::info("AutoShootDriveShoot pathname 2 {}", m_pathname2.c_str());

    AddCommands(
        IntakeDeploy(true),
        AutoWait(drivetrain),
        ShooterRun(Shooter::SHOOTERSPEED_FORWARD, shooter),
        frc2::ParallelCommandGroup{ AutoStop(drivetrain), ScoringAction(intake, fConv, vConv, shooter) },
        frc2::ParallelCommandGroup{ AutoDrivePath(m_pathname1.c_str(), true, drivetrain),
                                    IntakingAction(intake, fConv, vConv) },
        frc2::ParallelCommandGroup{ AutoDrivePath(m_pathname2.c_str(), false, drivetrain),
                                    ShooterRun(Shooter::SHOOTERSPEED_FORWARD, shooter) },
        frc2::ParallelCommandGroup{ AutoStop(drivetrain), ScoringAction(intake, fConv, vConv, shooter) });
}

bool AutoShootDriveShoot::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}