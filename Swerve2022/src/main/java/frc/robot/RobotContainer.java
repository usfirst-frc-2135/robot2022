// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.DTConsts;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
  private static RobotContainer m_robotContainer = new RobotContainer( );

  // The robot's subsystems
  public final Drivetrain       m_drivetrain     = new Drivetrain( );
  public final Intake           m_intake         = new Intake( );
  public final FloorConveyor    m_floorConveyor  = new FloorConveyor( );
  public final TowerConveyor    m_towerConveyor  = new TowerConveyor( );
  public final Shooter          m_shooter        = new Shooter( );
  public final Climber          m_climber        = new Climber( );
  public final Vision           m_vision         = new Vision( );
  public final LED              m_led            = new LED( );
  public final Pneumatics       m_pneumatics     = new Pneumatics( );
  public final Power            m_power          = new Power( );

  // Joysticks
  private final XboxController  m_driver         = new XboxController(0);
  private final XboxController  m_operator       = new XboxController(1);

  // A chooser for autonomous commands
  SendableChooser<Command>      m_chooser        = new SendableChooser<>( );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer( )
  {
    addSmartDashboardWidgets( );

    configureButtonBindings( );

    initDefaultCommands( );

    initAutonomousChooser( );
  }

  public static RobotContainer getInstance( )
  {
    return m_robotContainer;
  }

  private void addSmartDashboardWidgets( )
  {
    // Smartdashboard Subsystems

    // SmartDashboard Buttons
    SmartDashboard.putData("Auto1Ball1OppRight", new Auto1Ball1OppRight( ));
    SmartDashboard.putData("Auto1Ball2OppLeft", new Auto1Ball2OppLeft( ));
    SmartDashboard.putData("Auto1BallLimelight", new Auto1BallLimelight( ));
    SmartDashboard.putData("Auto3BallLeft", new Auto3BallLeft( ));
    SmartDashboard.putData("Auto3BallRight", new Auto3BallRight( ));
    SmartDashboard.putData("AutoDrive", new AutoDrive( ));
    SmartDashboard.putData("AutoDriveLimelightShoot", new AutoDriveLimelightShoot( ));
    SmartDashboard.putData("AutoDrivePath", new AutoDrivePath( ));
    SmartDashboard.putData("AutoDriveShoot", new AutoDriveShoot( ));
    SmartDashboard.putData("AutoPathSequence", new AutoPathSequence( ));
    SmartDashboard.putData("AutoShoot", new AutoShoot( ));
    SmartDashboard.putData("AutoShootDriveShoot", new AutoShootDriveShoot( ));
    SmartDashboard.putData("AutoStop", new AutoStop(m_drivetrain));
    SmartDashboard.putData("Climber0Stow", new Climber0Stow( ));
    SmartDashboard.putData("Climber1Deploy", new Climber1Deploy( ));
    SmartDashboard.putData("Climber2ClimbToL2", new Climber2ClimbToL2( ));
    SmartDashboard.putData("Climber3RotateToL3", new Climber3RotateToL3( ));
    SmartDashboard.putData("Climber5RotateIntoL3", new Climber5RotateIntoL3( ));
    SmartDashboard.putData("Climber6ClimbToL3", new Climber6ClimbToL3( ));
    SmartDashboard.putData("Climber7ClimbToL4", new Climber7ClimbToL4( ));
    SmartDashboard.putData("Climber8SettleToL4", new Climber8SettleToL4( ));
    SmartDashboard.putData("ClimberCalibrate", new ClimberCalibrate(m_climber));
    SmartDashboard.putData("ClimberFullClimb", new ClimberFullClimb( ));
    SmartDashboard.putData("ClimberL2ToL3", new ClimberL2ToL3( ));
    SmartDashboard.putData("ClimberL3ToL4", new ClimberL3ToL4( ));
    SmartDashboard.putData("ClimberMoveToHeight", new ClimberMoveToHeight(m_climber));
    SmartDashboard.putData("ClimberSetGatehook", new ClimberSetGatehook( ));
    SmartDashboard.putData("ClimberTimerOverride", new ClimberTimerOverride( ));
    SmartDashboard.putData("DriveLimelight", new DriveLimelight( ));
    SmartDashboard.putData("DriveMotorTest", new DriveMotorTest( ));
    SmartDashboard.putData("DriveResetSensors", new DriveResetSensors( ));
    SmartDashboard.putData("ExhaustingAction", new ExhaustingAction( ));
    SmartDashboard.putData("ExhaustingStop", new ExhaustingStop( ));
    SmartDashboard.putData("IntakingAction", new IntakingAction( ));
    SmartDashboard.putData("IntakingStop", new IntakingStop( ));
    SmartDashboard.putData("RobotInitialize", new RobotInitialize( ));
    SmartDashboard.putData("ShooterAimToggle", new ShooterAimToggle( ));
    SmartDashboard.putData("ShooterReverse", new ShooterReverse( ));
    SmartDashboard.putData("SimulateLimelight", new SimulateLimelight( ));
    SmartDashboard.putData("Dummy", new Dummy( ));
  }

  // Create a trigger object that monitors a joystick axis

  private class AxisTrigger extends Trigger
  {
    XboxController m_gamepad;
    Axis           m_axis;

    AxisTrigger(XboxController gamepad, Axis axis)
    {
      m_gamepad = gamepad;
      m_axis = axis;
    }

    @Override
    public boolean get( )
    {
      // This returns whether the trigger is active
      return (m_gamepad.getRawAxis(m_axis.value) > 0.5);
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by instantiating
   * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
   * {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings( )
  {
    // Create some buttons

  }

  // Configure the button bindings

  private void initDefaultCommands( )
  {
    // Configure default commands for these subsystems
    // m_drivetrain.setDefaultCommand(new DriveTeleop(m_drivetrain, m_driver, XboxController.Axis.kLeftY.value, XboxController.Axis.kRightX.value));
    m_drivetrain.setDefaultCommand(new DriveTeleop(m_drivetrain, m_driver));
  }

  // Configure autonomous sendable chooser

  private void initAutonomousChooser( )
  {
    m_chooser.addOption("AutoStop", new AutoStop(m_drivetrain));
    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand( ));

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public XboxController getDriver( )
  {
    return m_driver;
  }

  public XboxController getOperator( )
  {
    return m_operator;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand( )
  {
    // The selected command will be run in autonomous
    return m_chooser.getSelected( );
  }

  public static CANCoderConfiguration swerveCancoderConfig( )
  {
    CANCoderConfiguration config = new CANCoderConfiguration( );
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    config.sensorDirection = DTConsts.canCoderInvert;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    return config;
  }

}
