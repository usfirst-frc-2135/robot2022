// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "commands/Auto1BallLimelight.h"

#include "commands/AutoDriveLimelightShoot.h"
#include "commands/AutoDrivePath.h"
#include "commands/AutoStop.h"
#include "commands/AutoWait.h"
#include "commands/IntakeDeploy.h"
#include "commands/ScoringActionHighHub.h"
#include "commands/ScoringPrime.h"
#include "commands/ScoringStop.h"
#include "frc2135/RobotConfig.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <spdlog/spdlog.h>
#include <wpi/SmallString.h>

Auto1BallLimelight::Auto1BallLimelight(
    Drivetrain *drivetrain,
    Intake *intake,
    FloorConveyor *fConv,
    VerticalConveyor *vConv,
    Shooter *shooter,
    Vision *vision)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("Auto1BallLimelight");

    // Add your commands here, e.g.
    // AddCommands(FooCommand(), BarCommand());
    frc2135::RobotConfig *config = frc2135::RobotConfig::GetInstance();
    config->GetValueAsString("Auto1BallLimelight_path1", m_pathname1, "fenderToOffTarmac");
    config->GetValueAsString("Auto1BallLimelight_path2", m_pathname2, "shootingPosToOffTarmac");
    spdlog::info("Auto1BallLimelight pathname 1 {}", m_pathname1.c_str());
    spdlog::info("Auto1BallLimelight pathname 2 {}", m_pathname2.c_str());

    AddCommands( // Sequential command
        // Wait timer set in SmartDasboard
        AutoWait(drivetrain),
        //frc2::ParallelDeadlineGroup{ IntakeDeploy(true), AutoStop(drivetrain) },
        frc2::ParallelCommandGroup{
            frc2::ParallelDeadlineGroup{
                frc2::WaitUntilCommand([drivetrain] { return drivetrain->RamseteFollowerIsFinished(); }),
                AutoDrivePath(m_pathname1.c_str(), true, drivetrain) },
            ScoringPrime(shooter) },
        // Run limelight shooting routine for 3rd ball
        frc2::PrintCommand("Run limelight shooting routine for 3rd ball"),
        frc2::ConditionalCommand{ AutoDriveLimelightShoot(drivetrain, intake, fConv, vConv, shooter, vision),
                                  ScoringActionHighHub(2_s, intake, fConv, vConv, shooter),
                                  [drivetrain]
                                  {
                                      spdlog::info("Going to check limelight sanity");
                                      return drivetrain->LimelightSanityCheck(25, 25);
                                  } },
        frc2::ParallelDeadlineGroup{
            frc2::WaitUntilCommand([drivetrain] { return drivetrain->RamseteFollowerIsFinished(); }),
            AutoDrivePath(m_pathname2.c_str(), false, drivetrain),
            ScoringStop(intake, fConv, vConv, shooter) });
}

bool Auto1BallLimelight::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}