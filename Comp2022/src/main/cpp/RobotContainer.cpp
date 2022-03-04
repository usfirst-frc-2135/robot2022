// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "RobotContainer.h"

#include "commands/ClimberRun.h"
#include "frc2135/AxisButton.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ParallelRaceGroup.h>

RobotContainer *RobotContainer::m_robotContainer = NULL;

RobotContainer::RobotContainer() :
    m_autonomousCommand(
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        &m_drivetrain)
{
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Smartdashboard Subsystems

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD

    // SmartDashboard Subsystem Buttons
    frc::SmartDashboard::PutData("Intake Deploy: INTAKE_EXTEND", new IntakeDeploy(true));
    frc::SmartDashboard::PutData("Intake Deploy: INTAKE_STOW", new IntakeDeploy(false));
    frc::SmartDashboard::PutData("Intake Run: INTAKE_STOP", new IntakeRun(0, &m_intake));
    frc::SmartDashboard::PutData("Intake Run: INTAKE_ACQUIRE", new IntakeRun(1, &m_intake));
    frc::SmartDashboard::PutData("Intake Run: INTAKE_EXPEL", new IntakeRun(-1, &m_intake));
    frc::SmartDashboard::PutData("Floor Conveyor Run: FCONDIR_STOP", new FloorConveyorRun(0, &m_floorConv));
    frc::SmartDashboard::PutData("Floor Conveyor Run: FCONDIR_ACQUIRE", new FloorConveyorRun(1, &m_floorConv));
    frc::SmartDashboard::PutData("Floor Conveyor Run: FCONDIR_EXPEL", new FloorConveyorRun(-1, &m_floorConv));
    frc::SmartDashboard::PutData("Vertical Conv Run: VCONDIR_STOP", new VerticalConveyorRun(0, &m_vertConv));
    frc::SmartDashboard::PutData("Vertical Conv Run: VCONDIR_ACQUIRE", new VerticalConveyorRun(1, &m_vertConv));
    frc::SmartDashboard::PutData("Vertical Conv Run: VCONDIR_EXPEL", new VerticalConveyorRun(-1, &m_vertConv));
    frc::SmartDashboard::PutData("Shooter Aim: LIGHT_ON", new ShooterAim(true));
    frc::SmartDashboard::PutData("Shooter Aim: LIGHT_OFF", new ShooterAim(false));
    frc::SmartDashboard::PutData("Shooter Run: SHDIR_STOP", new ShooterRun(0, &m_shooter));
    frc::SmartDashboard::PutData("Shooter Run: SHDIR_SHOOT_LOW", new ShooterRun(1, &m_shooter));
    frc::SmartDashboard::PutData("Shooter Run: SHDIR_SHOOT_HIGH", new ShooterRun(2, &m_shooter));
    frc::SmartDashboard::PutData("LED Set", new LEDSet(LED::LEDCOLOR_DASH, &m_led));
    frc::SmartDashboard::PutData("Robot Initialize", new RobotInitialize());

    frc::SmartDashboard::PutData("Auto Drive Path: forward79", new AutoDrivePath("forward79", true, &m_drivetrain));
    frc::SmartDashboard::PutData("Auto Drive Path: backward79", new AutoDrivePath("backward79", true, &m_drivetrain));
    frc::SmartDashboard::PutData("Limelight Drive", new DriveLimelight(false, &m_drivetrain, &m_vision));

    frc::SmartDashboard::PutData("LEFT Drivetrain Motor Testing", new DrivetrainMotorTesting(true, &m_drivetrain));
    frc::SmartDashboard::PutData("RIGHT Drivetrain Motor Testing", new DrivetrainMotorTesting(false, &m_drivetrain));

    // Group commands
    frc::SmartDashboard::PutData("Intaking Action", new IntakingAction(&m_intake, &m_floorConv, &m_vertConv));
    frc::SmartDashboard::PutData("Intaking Stop", new IntakingStop(&m_intake, &m_floorConv, &m_vertConv));
    frc::SmartDashboard::PutData("Exhausting Action", new ExhaustingAction(&m_intake, &m_floorConv, &m_vertConv));
    frc::SmartDashboard::PutData("Exhausting Stop", new ExhaustingStop(&m_intake, &m_floorConv, &m_vertConv));
    frc::SmartDashboard::PutData(
        "Scoring Action Low Hub",
        new ScoringActionLowHub(10_s, &m_intake, &m_floorConv, &m_vertConv, &m_shooter));
    frc::SmartDashboard::PutData(
        "Scoring Action High Hub",
        new ScoringActionHighHub(10_s, &m_intake, &m_floorConv, &m_vertConv, &m_shooter));
    frc::SmartDashboard::PutData("Scoring Stop", new ScoringStop(&m_intake, &m_floorConv, &m_vertConv, &m_shooter));
    frc::SmartDashboard::PutData("Full Climb", new ClimberFullClimb(&m_climber));

    // Autonomous chooser routines
    frc::SmartDashboard::PutData("1:  Auto Drive", new AutoDrive(&m_drivetrain, &m_intake));
    frc::SmartDashboard::PutData("0: Auto Stop", new AutoStop(&m_drivetrain));
    frc::SmartDashboard::PutData(
        "2a: Auto Shoot",
        new AutoShoot(&m_drivetrain, &m_intake, &m_floorConv, &m_vertConv, &m_shooter));
    frc::SmartDashboard::PutData(
        "2b: Auto Shoot Drive",
        new AutoShootDrive(&m_drivetrain, &m_intake, &m_floorConv, &m_vertConv, &m_shooter));
    frc::SmartDashboard::PutData(
        "3a: Auto Shoot Drive Shoot",
        new AutoShootDriveShoot(&m_drivetrain, &m_intake, &m_floorConv, &m_vertConv, &m_shooter));
    frc::SmartDashboard::PutData(
        "Auto Drive LL Shoot",
        new AutoDriveLimelightShoot(&m_drivetrain, &m_intake, &m_floorConv, &m_vertConv, &m_shooter, &m_vision));

    // Test autonomous routines
    frc::SmartDashboard::PutData("Auto Path Sequence", new AutoPathSequence(&m_drivetrain));
    frc::SmartDashboard::PutData("Auto Drive Path: left79", new AutoDrivePath("left79", true, &m_drivetrain));
    frc::SmartDashboard::PutData(
        "Auto Drive Path: simCurvePath",
        new AutoDrivePath("simCurvePath", true, &m_drivetrain));

    ConfigureButtonBindings();

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT-COMMANDS
    m_drivetrain.SetDefaultCommand(DriveTeleop(&m_drivetrain));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT-COMMANDS

    m_climber.SetDefaultCommand(ClimberMoveHeight(Climber::NOCHANGE_HEIGHT, &m_climber));

    frc::SmartDashboard::PutData("Climber Calibrate", new ClimberCalibrate(&m_climber));
    frc::SmartDashboard::PutData(
        "Climber 0 STOW",
        new ClimberStow(&m_climber, &m_intake, &m_floorConv, &m_vertConv, &m_shooter));
    frc::SmartDashboard::PutData("Climber 1 DEPLOY", new ClimberDeploy(&m_climber));

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    m_chooser.AddOption("1: Auto Drive", new AutoDrive(&m_drivetrain, &m_intake));
    m_chooser.AddOption(
        "2a: Auto Shoot",
        new AutoShoot(&m_drivetrain, &m_intake, &m_floorConv, &m_vertConv, &m_shooter));
    m_chooser.AddOption(
        "2b: Auto Shoot Drive",
        new AutoShootDrive(&m_drivetrain, &m_intake, &m_floorConv, &m_vertConv, &m_shooter));
    m_chooser.AddOption(
        "3a: Auto Shoot Drive Shoot",
        new AutoShootDriveShoot(&m_drivetrain, &m_intake, &m_floorConv, &m_vertConv, &m_shooter));
    m_chooser.AddOption(
        "Auto Drive LL Shoot",
        new AutoDriveLimelightShoot(&m_drivetrain, &m_intake, &m_floorConv, &m_vertConv, &m_shooter, &m_vision));
    m_chooser.AddOption("Auto Path Sequence", new AutoPathSequence(&m_drivetrain));

    m_chooser.SetDefaultOption("0: Auto Stop", new AutoStop(&m_drivetrain));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

#ifndef __FRC_ROBORIO__
    frc2::CommandScheduler::GetInstance().Schedule(&m_simulateLimelightCommand);
#endif

    frc::SmartDashboard::PutData("Auto Mode", &m_chooser);
}

RobotContainer *RobotContainer::GetInstance()
{
    if (m_robotContainer == NULL)
    {
        m_robotContainer = new RobotContainer();
    }
    return (m_robotContainer);
}

//  axes: kLeftX = 0, kLeftY = 1, kLeftTrigger = 2, kRightTrigger = 3, kRightX = 4, kRightY = 5
//  btns: kA = 1, kB = 2, kX = 3, kY = 4, kLeftBumper = 5, kRightBumper = 6
//  btns: kBack = 7, kStart = 8, kStickLeft = 9, kStickRight = 10

void RobotContainer::ConfigureButtonBindings()
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS

    // Driver Controller Assignments
    // Driver - A, B, X, Y
    frc2::JoystickButton m_driverA{ &m_driverController, (int)frc::XboxController::Button::kA };
    m_driverA.WhileHeld(DriveQuickturn(), true);

    // Driver - Bumpers
    frc2::JoystickButton m_driverLeftBumper{ &m_driverController, (int)frc::XboxController::Button::kLeftBumper };
    m_driverLeftBumper.WhenPressed(IntakingAction(&m_intake, &m_floorConv, &m_vertConv), true);
    m_driverLeftBumper.WhenReleased(IntakingStop(&m_intake, &m_floorConv, &m_vertConv), true);

    frc2::JoystickButton m_driverRightBumper{ &m_driverController, (int)frc::XboxController::Button::kRightBumper };
    m_driverRightBumper.WhenPressed(ScoringActionLowHub(10_s, &m_intake, &m_floorConv, &m_vertConv, &m_shooter), true);
    m_driverRightBumper.WhenReleased(ScoringStop(&m_intake, &m_floorConv, &m_vertConv, &m_shooter), true);

    // Driver - Triggers
    frc2135::AxisButton m_driverRightTrigger(&m_driverController, (int)frc::XboxController::Axis::kRightTrigger);
    m_driverRightTrigger.WhileHeld(
        DriveLimelightShoot(&m_drivetrain, &m_intake, &m_floorConv, &m_vertConv, &m_shooter, &m_vision));
    m_driverRightTrigger.WhenReleased(ScoringStop(&m_intake, &m_floorConv, &m_vertConv, &m_shooter), true);

    // Driver - Start/back
    frc2::JoystickButton m_driverStart{ &m_driverController, (int)frc::XboxController::Button::kStart };
    m_driverStart.ToggleWhenPressed(ShooterAim(true));

    // Operator Controller Assignments
    // Operator - A, B, X, Y
    frc2::JoystickButton m_operA{ &m_operatorController, (int)frc::XboxController::Button::kA };
    m_operA.WhenPressed(IntakeDeploy(true), true);
    frc2::JoystickButton m_operB{ &m_operatorController, (int)frc::XboxController::Button::kB };
    m_operB.WhenPressed(ExhaustingAction(&m_intake, &m_floorConv, &m_vertConv), true);
    m_operB.WhenReleased(ExhaustingStop(&m_intake, &m_floorConv, &m_vertConv), true);
    frc2::JoystickButton m_operX{ &m_operatorController, (int)frc::XboxController::Button::kX };
    m_operX.WhenPressed(ScoringStop(&m_intake, &m_floorConv, &m_vertConv, &m_shooter), true);
    frc2::JoystickButton m_operY{ &m_operatorController, (int)frc::XboxController::Button::kY };
    m_operY.WhenPressed(IntakeDeploy(false), true);

    // Operator - POV buttons
    frc2::POVButton m_operDown{ &m_operatorController, 180, 0 };
    m_operDown.WhenPressed(ClimberClimbToL2(&m_climber), true);
    frc2::POVButton m_operUp{ &m_operatorController, 0, 0 };
    m_operUp.WhenPressed(ClimberDeploy(&m_climber), true);
    frc2::POVButton m_operLeft{ &m_operatorController, 270, 0 };
    m_operLeft.WhenPressed(ClimberRotateToL3(&m_climber), true);
    frc2::POVButton m_operRight{ &m_operatorController, 90, 0 };
    m_operRight.WhenPressed(ClimberRotateIntoL3(&m_climber), true);

    // Operator Bumpers
    frc2::JoystickButton m_operLeftBumper{ &m_operatorController, (int)frc::XboxController::Button::kLeftBumper };
    m_operLeftBumper.WhenPressed(IntakingAction(&m_intake, &m_floorConv, &m_vertConv), true);
    m_operLeftBumper.WhenReleased(IntakingStop(&m_intake, &m_floorConv, &m_vertConv), true);
    frc2::JoystickButton m_operRightBumper{ &m_operatorController, (int)frc::XboxController::Button::kRightBumper };
    m_operRightBumper.WhenPressed(ScoringPrime(&m_shooter), true);

    // Operator Start/Back
    frc2::JoystickButton m_operStart{ &m_operatorController, (int)frc::XboxController::Button::kStart };
    m_operStart.ToggleWhenPressed(ClimberRun(&m_climber), true);
    frc2::JoystickButton m_operBack{ &m_operatorController, (int)frc::XboxController::Button::kBack };
    m_operBack.WhenPressed(ClimberFullClimb(&m_climber), true);
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

frc::XboxController *RobotContainer::getDriverController()
{
    return &m_driverController;
}
frc::XboxController *RobotContainer::getOperatorController()
{
    return &m_operatorController;
}

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    // The selected command will be run in autonomous
    return m_chooser.GetSelected();
}
