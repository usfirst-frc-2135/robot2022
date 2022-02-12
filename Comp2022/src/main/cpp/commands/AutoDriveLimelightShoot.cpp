// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "commands/AutoDriveLimelightShoot.h"

#include "commands/AutoDrivePath.h"
#include "commands/AutoPathSequence.h"
#include "commands/AutoStop.h"
#include "commands/AutoWait.h"
#include "commands/DriveLimelight.h"
#include "commands/IntakeDeploy.h"
#include "commands/ScoringAction.h"
#include "commands/ScoringPrime.h"
#include "frc2135/RobotConfig.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <spdlog/spdlog.h>
#include <wpi/SmallString.h>

AutoDriveLimelightShoot::AutoDriveLimelightShoot(
    Drivetrain *drivetrain,
    Intake *intake,
    FloorConveyor *fConv,
    VerticalConveyor *vConv,
    Shooter *shooter,
    Vision *vision)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("AutoDriveLimelightShoot");

    // Add your commands here, e.g.
    // AddCommands(FooCommand(), BarCommand());
    frc2135::RobotConfig *config = frc2135::RobotConfig::GetInstance();
    config->GetValueAsString("AutoDriveLimelightShoot_path", m_pathname, "driveForward46");
    spdlog::info("AutoDriveLimelightShoot pathname {}", m_pathname.c_str());

    AddCommands(
        IntakeDeploy(true),
        AutoWait(drivetrain),
        AutoDrivePath(true, m_pathname.c_str(), true, drivetrain),
        //drive backwards until target is valid
        frc2::ParallelCommandGroup{ DriveLimelight(true, drivetrain, vision), ScoringPrime(shooter) },
        frc2::ParallelCommandGroup{ DriveLimelight(false, drivetrain, vision),
                                    ScoringAction(intake, fConv, vConv, shooter) }
        // drive limelight servo instead of drive limelight in parallel with scoringaction
    );
}

bool AutoDriveLimelightShoot::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}