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

#include "commands/AutoDrivePath.h"
#include "commands/AutoStop.h"
#include "commands/AutoWait.h"
#include "commands/IntakeDeploy.h"
#include "commands/ScoringActionHighHub.h"
#include "commands/ScoringPrime.h"
#include "commands/ScoringStop.h"
#include "frc2135/RobotConfig.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ParallelDeadlineGroup.h>
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

    frc2135::RobotConfig *config = frc2135::RobotConfig::GetInstance();
    config->GetValueAsString("AutoShoot_path", m_pathname, "startToShootingPos");
    spdlog::info("AutoDriveShoot pathname {}", m_pathname.c_str());

    // Add your commands here, e.g.
    // AddCommands(FooCommand(), BarCommand());
    AddCommands( // Sequential command
        // Wait timer set in SmartDasboard
        AutoWait(drivetrain, 1),
        frc2::ParallelDeadlineGroup{ IntakeDeploy(true), AutoStop(drivetrain) },
        frc2::ParallelCommandGroup{
            frc2::ParallelDeadlineGroup{
                frc2::WaitUntilCommand([drivetrain] { return drivetrain->RamseteFollowerIsFinished(); }),
                AutoDrivePath(m_pathname.c_str(), true, drivetrain) },
            ScoringPrime(shooter) },
        frc2::ParallelDeadlineGroup{ ScoringActionHighHub(5_s, intake, fConv, vConv, shooter), AutoStop(drivetrain) },
        ScoringStop(intake, fConv, vConv, shooter));
}

bool AutoShoot::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}