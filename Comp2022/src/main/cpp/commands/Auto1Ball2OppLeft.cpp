// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "commands/Auto1Ball2OppLeft.h"

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
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <spdlog/spdlog.h>
#include <wpi/SmallString.h>

Auto1Ball2OppLeft::Auto1Ball2OppLeft(
    Drivetrain *drivetrain,
    Intake *intake,
    FloorConveyor *fConv,
    VerticalConveyor *vConv,
    Shooter *shooter)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("Auto1Ball2OppLeft");

    // Add your commands here, e.g.
    // AddCommands(FooCommand(), BarCommand());
    frc2135::RobotConfig *config = frc2135::RobotConfig::GetInstance();
    config->GetValueAsString("Auto1Ball2OppLeft_path1", m_pathname1, "startToShootingPos");
    config->GetValueAsString("Auto1Ball2OppLeft_path2", m_pathname2, "shootingPosToLeftOppBall1");
    config->GetValueAsString("Auto1Ball2OppLeft_path3", m_pathname3, "leftOppBall1ToBall2");
    config->GetValueAsString("Auto1Ball2OppLeft_path4", m_pathname4, "leftBallToLeftShootingPos");
    spdlog::info("Auto1Ball2OppLeft pathname 1 {}", m_pathname1.c_str());
    spdlog::info("Auto1Ball2OppLeft pathname 2 {}", m_pathname2.c_str());
    spdlog::info("Auto1Ball2OppLeft pathname 3 {}", m_pathname3.c_str());
    spdlog::info("Auto1Ball2OppLeft pathname 4 {}", m_pathname4.c_str());

    AddCommands( // Sequential command
        frc2::PrintCommand("AUTO 1 BALL 2 OPP LEFT - START"),
        // Wait timer set in SmartDasboard
        frc2::PrintCommand("WAIT"),
        AutoWait(drivetrain),
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
        // Drive to 1st opponent's ball and intake
        frc2::PrintCommand("Drive to 1st opponent's ball and intake"),
        frc2::ParallelDeadlineGroup{
            frc2::WaitUntilCommand([drivetrain] { return drivetrain->RamseteFollowerIsFinished(); }),
            AutoDrivePath(m_pathname2.c_str(), false, drivetrain),
            IntakingAction(intake, fConv, vConv) },
        // Second wait timer set in SmartDasboard
        frc2::PrintCommand("WAIT"),
        AutoWait(drivetrain),
        // Drive to 2nd opponent's ball and intake
        frc2::PrintCommand("Drive to 2nd opponent's ball and intake"),
        frc2::ParallelDeadlineGroup{
            frc2::WaitUntilCommand([drivetrain] { return drivetrain->RamseteFollowerIsFinished(); }),
            AutoDrivePath(m_pathname3.c_str(), false, drivetrain),
            ScoringPrime(shooter),
            IntakingAction(intake, fConv, vConv) },
        // Stow intake
        frc2::PrintCommand("Stow intake"),
        frc2::ParallelDeadlineGroup{ IntakeDeploy(false), AutoStop(drivetrain) },
        /* PUT IN CONDITIONAL COMMAND: LITERALLY IF THIS RETURN THIS OR LIKE RETURN THE BOOLEAN // Turn and drive to a shooting position
        frc2::PrintCommand("Turn and drive to a shooting position"),
        frc2::ParallelCommandGroup{
            frc2::ParallelDeadlineGroup{
                frc2::WaitUntilCommand([drivetrain] { return drivetrain->RamseteFollowerIsFinished(); }),
                AutoDrivePath(m_pathname4.c_str(), false, drivetrain) },
            ScoringPrime(shooter) },*/
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

bool Auto1Ball2OppLeft::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}