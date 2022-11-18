// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
    SmartDashboard.putData("DriveLimelight", new DriveLimelight(m_drivetrain, m_vision, false));
    SmartDashboard.putData("DriveMotorTest", new DriveMotorTest(m_drivetrain, true));
    SmartDashboard.putData("DriveResetSensors", new DriveResetSensors(m_drivetrain));
    SmartDashboard.putData("ExhaustingAction", new ExhaustingAction(m_intake, m_floorConveyor, m_towerConveyor));
    SmartDashboard.putData("ExhaustingStop", new ExhaustingStop(m_intake, m_floorConveyor, m_towerConveyor));
    SmartDashboard.putData("IntakingAction", new IntakingAction(m_intake, m_floorConveyor, m_towerConveyor));
    SmartDashboard.putData("IntakingStop", new IntakingStop(m_intake, m_floorConveyor, m_towerConveyor));
    SmartDashboard.putData("RobotInitialize", new RobotInitialize( ));
    SmartDashboard.putData("ShooterAimToggle", new ShooterAimToggle( ));
    SmartDashboard.putData("ShooterReverse", new ShooterReverse(m_shooter));
    //SmartDashboard.putData("SimulateLimelight", new SimulateLimelight( ));
    //SmartDashboard.putData("Dummy", new Dummy( ));
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
    ///////////////////////////////////////////////////////
    // Driver Controller Assignments
    final JoystickButton driverA = new JoystickButton(m_driver, XboxController.Button.kA.value);
    final JoystickButton driverB = new JoystickButton(m_driver, XboxController.Button.kB.value);
    final JoystickButton driverX = new JoystickButton(m_driver, XboxController.Button.kX.value);
    final JoystickButton driverY = new JoystickButton(m_driver, XboxController.Button.kY.value);
    final JoystickButton driverLeftBumper = new JoystickButton(m_driver, XboxController.Button.kLeftBumper.value);
    final JoystickButton driverRightBumper = new JoystickButton(m_driver, XboxController.Button.kRightBumper.value);
    final JoystickButton driverBack = new JoystickButton(m_driver, XboxController.Button.kBack.value);
    final JoystickButton driverStart = new JoystickButton(m_driver, XboxController.Button.kStart.value);
    final POVButton driverUp = new POVButton(m_driver, 0);
    final POVButton driverRight = new POVButton(m_driver, 90);
    final POVButton driverDown = new POVButton(m_driver, 180);
    final POVButton driverLeft = new POVButton(m_driver, 270);
    // @formatter:off
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    final AxisTrigger driverLeftTrigger = new AxisTrigger(m_driver, XboxController.Axis.kLeftTrigger);
    final AxisTrigger driverRightTrigger = new AxisTrigger(m_driver, XboxController.Axis.kRightTrigger);
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    // final AxisTrigger driverLeftTrigger = new AxisTrigger(m_driver, XboxController.Axis.kRightX);
    // final AxisTrigger driverRightTrigger = new AxisTrigger(m_driver, XboxController.Axis.kRightY);
    // @formatter:on

    // Driver - A, B, X, Y
    driverA.whileHeld(new DriveQuickturn(m_drivetrain), true);
    driverB.whenPressed(new Dummy(XboxController.Button.kB.value), true);
    driverX.whenPressed(new Dummy(XboxController.Button.kX.value), true);
    driverY.whenPressed(new Dummy(XboxController.Button.kY.value), true);

    // Driver - Bumpers, start, back
    driverLeftBumper.whenPressed(new IntakingAction(m_intake, m_floorConveyor, m_towerConveyor), true);
    driverLeftBumper.whenReleased(new IntakingStop(m_intake, m_floorConveyor, m_towerConveyor), true);
    driverRightBumper.whenPressed(new ScoringActionLowerHub(m_intake, m_floorConveyor, m_towerConveyor, m_shooter, 10.0), true);
    driverRightBumper.whenReleased(new ScoringStop(m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision), true);
    driverBack.whenPressed(new Dummy(XboxController.Button.kBack.value), true);
    driverStart.whenPressed(new VisionOn(m_vision, VIRequests.VISION_TOGGLE), true);

    // Driver - POV buttons
    driverUp.whenPressed(new Dummy(0), true);
    driverRight.whenPressed(new Dummy(90), true);
    driverDown.whenPressed(new Dummy(180), true);
    driverLeft.whenPressed(new Dummy(270), true);

    // Driver - Triggers
    driverLeftTrigger.whenActive(new Dummy(256));
    driverRightTrigger
        .whenActive(new DriveLimelightShoot(m_drivetrain, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    driverRightTrigger
        .whenInactive(new DriveLimelightStop(m_drivetrain, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));

    ///////////////////////////////////////////////////////
    // Operator Controller Assignments
    final JoystickButton operA = new JoystickButton(m_operator, XboxController.Button.kA.value);
    final JoystickButton operB = new JoystickButton(m_operator, XboxController.Button.kB.value);
    final JoystickButton operX = new JoystickButton(m_operator, XboxController.Button.kX.value);
    final JoystickButton operY = new JoystickButton(m_operator, XboxController.Button.kY.value);
    final JoystickButton operLeftBumper = new JoystickButton(m_operator, XboxController.Button.kLeftBumper.value);
    final JoystickButton operRightBumper = new JoystickButton(m_operator, XboxController.Button.kRightBumper.value);
    final JoystickButton operBack = new JoystickButton(m_operator, XboxController.Button.kBack.value);
    final JoystickButton operStart = new JoystickButton(m_operator, XboxController.Button.kStart.value);
    final POVButton operUp = new POVButton(m_operator, 0);
    final POVButton operRight = new POVButton(m_operator, 90);
    final POVButton operDown = new POVButton(m_operator, 180);
    final POVButton operLeft = new POVButton(m_operator, 270);
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    final AxisTrigger operLeftTrigger = new AxisTrigger(m_operator, XboxController.Axis.kLeftTrigger);
    final AxisTrigger operRightTrigger = new AxisTrigger(m_operator, XboxController.Axis.kRightTrigger);
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    // final AxisTrigger operLeftTrigger = new AxisTrigger(m_operator, XboxController.Axis.rightX);
    // final AxisTrigger operRightTrigger = new AxisTrigger(m_operator, XboxController.Axis.rightY);

    // Operator - A, B, X, Y
    operA.whenPressed(new IntakeDeploy(m_intake, false), true);
    operB.whenPressed(new ExhaustingAction(m_intake, m_floorConveyor, m_towerConveyor), true);
    operB.whenReleased(new ExhaustingStop(m_intake, m_floorConveyor, m_towerConveyor), true);
    operX.whenPressed(new ScoringStop(m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision), true);
    //operY.whenPressed(new ClimberTimerOverride(m_climber, m_operator, XboxController.Button.kY), true);

    // Operator - Bumpers, start, back
    operLeftBumper.whenPressed(new IntakingAction(m_intake, m_floorConveyor, m_towerConveyor), true);
    operLeftBumper.whenReleased(new IntakingStop(m_intake, m_floorConveyor, m_towerConveyor), true);
    operRightBumper.whenPressed(new ScoringPrime(m_shooter, m_vision), true);
    //operBack.whenPressed(new ClimberFullClimb(m_climber, m_operator, XboxController.Button.kY), true);
    //operStart.toggleWhenPressed(new ClimberRun(m_climber, m_operator), true);

    // Operator - POV buttons
    // operUp.whenPressed(new Climber1Deploy(m_climber, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_drivetrain), true);
    // operRight.whenPressed(new ClimberL3ToL4(m_climber, m_intake, m_floorConveyor, m_towerConveyor, m_shooter), true);
    // operDown.whenPressed(new Climber0Stow(m_climber, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_drivetrain), true);
    // operLeft.whenPressed(new ClimberL2ToL3(m_climber, m_intake, m_floorConveyor, m_towerConveyor, m_shooter), true);

    // Operator Left/Right Trigger
    //operLeftTrigger.whenActive(new ClimberCalibrate(m_climber), true);
    operRightTrigger.whileActiveContinuous(new ShooterReverse(m_shooter), true);
  }

  // Configure the button bindings

  private void initDefaultCommands( )
  {
    // Configure default commands for these subsystems
    m_drivetrain.setDefaultCommand(
        new DriveTeleop(m_drivetrain, m_driver, XboxController.Axis.kLeftY.value, XboxController.Axis.kRightX.value));

    // Configure autonomous sendable chooser
  }

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
