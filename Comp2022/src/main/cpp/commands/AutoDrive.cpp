// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "commands/AutoDrive.h"

#include "commands/AutoDrivePath.h"
#include "commands/AutoStop.h"
#include "commands/AutoWait.h"
#include "commands/IntakeDeploy.h"
#include "frc2135/RobotConfig.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/WaitUntilCommand.h>
#include <spdlog/spdlog.h>
#include <wpi/SmallString.h>

AutoDrive::AutoDrive(Drivetrain *drivetrain, Intake *intake)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("AutoDrive");

    // Add your commands here, e.g.
    // AddCommands(FooCommand(), BarCommand());
    frc2135::RobotConfig *config = frc2135::RobotConfig::GetInstance();
    config->GetValueAsString("AutoDrive_path", m_pathname, "startToOffTarmac");
    spdlog::info("AutoDrive pathname {}", m_pathname.c_str());

    AddCommands( // Sequential command
        // frc2::ParallelDeadlineGroup{ IntakeDeploy(true), AutoStop(drivetrain) },
        AutoWait(drivetrain),
        frc2::ParallelDeadlineGroup{
            frc2::WaitUntilCommand([drivetrain] { return drivetrain->RamseteFollowerIsFinished(); }),
            AutoDrivePath(m_pathname.c_str(), true, drivetrain) },
        AutoStop(drivetrain));
}

bool AutoDrive::RunsWhenDisabled() const
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
}