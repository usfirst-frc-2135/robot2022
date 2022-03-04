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
#include "commands/AutoStop.h"
#include "commands/AutoWait.h"
#include "commands/IntakeDeploy.h"
#include "commands/IntakingAction.h"
#include "commands/IntakingStop.h"
#include "commands/ScoringActionHighHub.h"
#include "commands/ScoringPrime.h"
#include "commands/ScoringStop.h"
#include "frc2135/RobotConfig.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
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
    config->GetValueAsString("AutoShootDriveShoot_path1", m_pathname1, "startToShootingPos");
    config->GetValueAsString("AutoShootDriveShoot_path2", m_pathname2, "shootingPosToBall");
    config->GetValueAsString("AutoShootDriveShoot_path3", m_pathname3, "ballToShootingPos");
    config->GetValueAsString("AutoShootDriveShoot_path4", m_pathname4, "shootingPosToOffTarmac");
    spdlog::info("AutoShootDriveShoot pathname 1 {}", m_pathname1.c_str());
    spdlog::info("AutoShootDriveShoot pathname 2 {}", m_pathname2.c_str());
    spdlog::info("AutoShootDriveShoot pathname 3 {}", m_pathname3.c_str());
    spdlog::info("AutoShootDriveShoot pathname 4 {}", m_pathname4.c_str());

    AddCommands( // Sequential command
        frc2::ParallelDeadlineGroup{ IntakeDeploy(true), AutoStop(drivetrain) },
        AutoWait(drivetrain),
        frc2::ParallelCommandGroup{
            frc2::ParallelDeadlineGroup{
                frc2::WaitUntilCommand([drivetrain] { return drivetrain->RamseteFollowerIsFinished(); }),
                AutoDrivePath(m_pathname1.c_str(), true, drivetrain) },
            ScoringPrime(shooter) },
        frc2::ParallelDeadlineGroup{ ScoringActionHighHub(1_s, intake, fConv, vConv, shooter), AutoStop(drivetrain) },
        frc2::ParallelDeadlineGroup{
            frc2::ParallelDeadlineGroup{
                frc2::WaitUntilCommand([drivetrain] { return drivetrain->RamseteFollowerIsFinished(); }),
                AutoDrivePath(m_pathname2.c_str(), false, drivetrain) },
            IntakingAction(intake, fConv, vConv),
            ScoringPrime(shooter) },
        frc2::ParallelCommandGroup{
            frc2::ParallelDeadlineGroup{
                frc2::WaitUntilCommand([drivetrain] { return drivetrain->RamseteFollowerIsFinished(); }),
                AutoDrivePath(m_pathname3.c_str(), false, drivetrain) },
            IntakingStop(intake, fConv, vConv) },
        frc2::ParallelDeadlineGroup{ ScoringActionHighHub(2_s, intake, fConv, vConv, shooter), AutoStop(drivetrain) },
        IntakeDeploy(false),
        frc2::WaitCommand(1_s),
        frc2::ParallelCommandGroup{
            frc2::ParallelDeadlineGroup{
                frc2::WaitUntilCommand([drivetrain] { return drivetrain->RamseteFollowerIsFinished(); }),
                AutoDrivePath(m_pathname4.c_str(), false, drivetrain) },
            ScoringStop(intake, fConv, vConv, shooter) });
}

bool AutoShootDriveShoot::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}