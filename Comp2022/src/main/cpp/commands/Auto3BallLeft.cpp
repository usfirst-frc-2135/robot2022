// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "commands/Auto3BallLeft.h"

#include "commands/AutoDrivePath.h"
#include "commands/AutoStop.h"
#include "commands/AutoWait.h"
#include "commands/IntakeDeploy.h"
#include "commands/IntakingAction.h"
#include "commands/IntakingStop.h"
#include "commands/ScoringActionHighHub.h"
#include "commands/ScoringActionLowHub.h"
#include "commands/ScoringPrime.h"
#include "commands/ScoringStop.h"
#include "frc2135/RobotConfig.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <spdlog/spdlog.h>
#include <wpi/SmallString.h>

Auto3BallLeft::Auto3BallLeft(
    Drivetrain *drivetrain,
    Intake *intake,
    FloorConveyor *fConv,
    VerticalConveyor *vConv,
    Shooter *shooter)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("Auto3BallLeft");

    // Add your commands here, e.g.
    // AddCommands(FooCommand(), BarCommand());
    frc2135::RobotConfig *config = frc2135::RobotConfig::GetInstance();
    config->GetValueAsString("Auto3BallLeft_path1", m_pathname1, "startToShootingPos");
    config->GetValueAsString("Auto3BallLeft_path2", m_pathname2, "shootingPosToBall");
    config->GetValueAsString("Auto3BallLeft_path3", m_pathname3, "ballToShootingPos");
    config->GetValueAsString("Auto3BallLeft_path4", m_pathname4, "shootingPosToLeftBall");
    config->GetValueAsString("Auto3BallLeft_path5", m_pathname5, "leftBallToLeftShootingPos");
    spdlog::info("Auto3BallLeft pathname 1 {}", m_pathname1.c_str());
    spdlog::info("Auto3BallLeft pathname 2 {}", m_pathname2.c_str());
    spdlog::info("Auto3BallLeft pathname 3 {}", m_pathname3.c_str());
    spdlog::info("Auto3BallLeft pathname 4 {}", m_pathname4.c_str());
    spdlog::info("Auto3BallLeft pathname 5 {}", m_pathname5.c_str());

    AddCommands( // Sequential command
        frc2::PrintCommand("AUTO 3 BALL LEFT - START"),
        // Wait timer set in SmartDasboard
        AutoWait(drivetrain, 1),
        // Deploy intake
        frc2::PrintCommand("Deploy intake"),
        frc2::ParallelDeadlineGroup{ IntakeDeploy(true), AutoStop(drivetrain) },
        // Drive to a shooting position
        frc2::PrintCommand("Drive to a shooting position"),
        frc2::ParallelDeadlineGroup{
            frc2::WaitUntilCommand([drivetrain] { return drivetrain->RamseteFollowerIsFinished(); }),
            AutoDrivePath(m_pathname1.c_str(), true, drivetrain),
            ScoringPrime(shooter) },
        // Shoot preloaded ball
        frc2::PrintCommand("Shoot preloaded ball"),
        frc2::ParallelDeadlineGroup{ ScoringActionHighHub(1_s, intake, fConv, vConv, shooter), AutoStop(drivetrain) },
        // Drive to 2nd ball and intake
        frc2::PrintCommand("Drive to 2nd ball and intake"),
        frc2::ParallelDeadlineGroup{
            frc2::WaitUntilCommand([drivetrain] { return drivetrain->RamseteFollowerIsFinished(); }),
            AutoDrivePath(m_pathname2.c_str(), false, drivetrain),
            IntakingAction(intake, fConv, vConv),
            ScoringPrime(shooter) },
        frc2::PrintCommand("Drive to a shooting position"),
        // Drive to a shooting position
        frc2::ParallelDeadlineGroup{
            frc2::WaitUntilCommand([drivetrain] { return drivetrain->RamseteFollowerIsFinished(); }),
            AutoDrivePath(m_pathname3.c_str(), false, drivetrain),
            IntakingAction(intake, fConv, vConv) },
        // Shoot 2nd ball
        frc2::PrintCommand("Shoot 2nd ball"),
        frc2::ParallelDeadlineGroup{ ScoringActionHighHub(2_s, intake, fConv, vConv, shooter), AutoStop(drivetrain) },
        // Drive to opponent's ball and intake
        frc2::PrintCommand("Drive to opponent's ball and intake"),
        frc2::ParallelDeadlineGroup{
            frc2::WaitUntilCommand([drivetrain] { return drivetrain->RamseteFollowerIsFinished(); }),
            AutoDrivePath(m_pathname4.c_str(), false, drivetrain),
            ScoringPrime(shooter),
            IntakingAction(intake, fConv, vConv) },
        // Stow intake
        frc2::PrintCommand("Stow intake"),
        frc2::ParallelDeadlineGroup{ IntakeDeploy(false), AutoStop(drivetrain) },
        // Shoot opponent's ball
        frc2::ConditionalCommand{ frc2::ParallelDeadlineGroup{ ScoringActionLowHub(2_s, intake, fConv, vConv, shooter),
                                                               AutoStop(drivetrain),
                                                               frc2::PrintCommand("Shoot opponent's ball") },
                                  AutoStop(drivetrain),
                                  [this] { return frc::SmartDashboard::GetBoolean("AUTO_ShootOppBall", false); } },
        // Stop shooting and driving
        frc2::PrintCommand("Stop shooting"),
        frc2::ParallelDeadlineGroup{ ScoringStop(intake, fConv, vConv, shooter), AutoStop(drivetrain) },
        frc2::PrintCommand("AUTO 3 BALL LEFT - END"));
}

bool Auto3BallLeft::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}