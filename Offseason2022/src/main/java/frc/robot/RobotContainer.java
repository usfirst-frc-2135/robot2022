
// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FCConsts.FCMode;
import frc.robot.Constants.INConsts.INMode;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.Constants.SHConsts.Mode;
import frc.robot.Constants.TCConsts.TCMode;
import frc.robot.commands.Auto1Ball1OppRight;
import frc.robot.commands.Auto1Ball2OppLeft;
import frc.robot.commands.Auto1BallLimelight;
import frc.robot.commands.Auto3BallLeft;
import frc.robot.commands.Auto3BallRight;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoDriveLimelightShoot;
import frc.robot.commands.AutoDrivePath;
import frc.robot.commands.AutoDriveShoot;
import frc.robot.commands.AutoPathSequence;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoShootDriveShoot;
import frc.robot.commands.AutoShootLowHub;
import frc.robot.commands.AutoStop;
import frc.robot.commands.Climber0Stow;
import frc.robot.commands.Climber1Deploy;
import frc.robot.commands.Climber2ClimbToL2;
import frc.robot.commands.Climber3RotateToL3;
import frc.robot.commands.Climber5RotateIntoL3;
import frc.robot.commands.Climber6ClimbToL3;
import frc.robot.commands.Climber7ClimbToL4;
import frc.robot.commands.Climber8SettleToL4;
import frc.robot.commands.ClimberCalibrate;
import frc.robot.commands.ClimberFullClimb;
import frc.robot.commands.ClimberL2ToL3;
import frc.robot.commands.ClimberL3ToL4;
import frc.robot.commands.ClimberRun;
import frc.robot.commands.ClimberSetGatehook;
import frc.robot.commands.ClimberTimerOverride;
import frc.robot.commands.DriveLimelight;
import frc.robot.commands.DriveLimelightShoot;
import frc.robot.commands.DriveLimelightStop;
import frc.robot.commands.DriveMotorTest;
import frc.robot.commands.DriveQuickturn;
import frc.robot.commands.DriveResetSensors;
import frc.robot.commands.DriveSlowMode;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.Dummy;
import frc.robot.commands.ExhaustingAction;
import frc.robot.commands.ExhaustingStop;
import frc.robot.commands.FloorConveyorRun;
import frc.robot.commands.IntakeDeploy;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.IntakingAction;
import frc.robot.commands.IntakingStop;
import frc.robot.commands.LEDSet;
import frc.robot.commands.RobotInitialize;
import frc.robot.commands.ScoringActionHighHub;
import frc.robot.commands.ScoringActionLowHub;
import frc.robot.commands.ScoringPrime;
import frc.robot.commands.ScoringStop;
import frc.robot.commands.ShooterReverse;
import frc.robot.commands.ShooterRun;
import frc.robot.commands.SimulateLimelight;
import frc.robot.commands.TowerConveyorRun;
import frc.robot.commands.VisionOn;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Power;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TowerConveyor;
import frc.robot.subsystems.Vision;

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
    SmartDashboard.putData("AutoShootLowHub", new AutoShootLowHub( ));
    SmartDashboard.putData("AutoStop", new AutoStop(m_drivetrain));

    SmartDashboard.putData("Climber0Stow",
        new Climber0Stow(m_climber, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_drivetrain));
    SmartDashboard.putData("Climber1Deploy",
        new Climber1Deploy(m_climber, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_drivetrain));
    SmartDashboard.putData("Climber2ClimbToL2", new Climber2ClimbToL2(m_climber));
    SmartDashboard.putData("Climber3RotateToL3", new Climber3RotateToL3(m_climber));
    SmartDashboard.putData("Climber5RotateIntoL3", new Climber5RotateIntoL3(m_climber));
    SmartDashboard.putData("Climber6ClimbToL3", new Climber6ClimbToL3(m_climber));
    SmartDashboard.putData("Climber7ClimbToL4", new Climber7ClimbToL4(m_climber));
    SmartDashboard.putData("Climber8SettleToL4", new Climber8SettleToL4(m_climber));
    SmartDashboard.putData("ClimberCalibrate", new ClimberCalibrate(m_climber));
    SmartDashboard.putData("ClimberFullClimb", new ClimberFullClimb(m_climber));
    SmartDashboard.putData("ClimberL2ToL3", new ClimberL2ToL3(m_climber, m_intake, m_floorConveyor, m_towerConveyor, m_shooter));
    SmartDashboard.putData("ClimberL3ToL4", new ClimberL3ToL4(m_climber, m_intake, m_floorConveyor, m_towerConveyor, m_shooter));
    SmartDashboard.putData("ClimberSetGatehook", new ClimberSetGatehook(m_climber, false));
    SmartDashboard.putData("ClimberTimerOverride", new ClimberTimerOverride(m_climber, m_operator, XboxController.Button.kY));
    SmartDashboard.putData("DriveLimelight", new DriveLimelight(m_drivetrain, m_vision, false));
    SmartDashboard.putData("DriveLimelightStop", new DriveLimelightStop(m_drivetrain));
    SmartDashboard.putData("DriveLimelightShoot",
        new DriveLimelightShoot(m_drivetrain, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    SmartDashboard.putData("DriveMotorTest", new DriveMotorTest(m_drivetrain, true));
    SmartDashboard.putData("DriveMotorTest", new DriveQuickturn(m_drivetrain));
    SmartDashboard.putData("DriveResetSensors", new DriveResetSensors(m_drivetrain));
    SmartDashboard.putData("DriveSlowMode", new DriveSlowMode(m_drivetrain, false));

    SmartDashboard.putData("ExhaustingAction", new ExhaustingAction(m_intake, m_floorConveyor, m_towerConveyor));
    SmartDashboard.putData("ExhaustingStop", new ExhaustingStop(m_intake, m_floorConveyor, m_towerConveyor));

    SmartDashboard.putData("Fconveyor-STOP", new FloorConveyorRun(m_floorConveyor, FCMode.FCONVEYOR_STOP));
    SmartDashboard.putData("Fconveyor-ACQUIRE", new FloorConveyorRun(m_floorConveyor, FCMode.FCONVEYOR_ACQUIRE));
    SmartDashboard.putData("Fconveyor-EXPEL", new FloorConveyorRun(m_floorConveyor, FCMode.FCONVEYOR_EXPEL));
    SmartDashboard.putData("Fconveyor-EXPELFAST", new FloorConveyorRun(m_floorConveyor, FCMode.FCONVEYOR_EXPEL_FAST));

    SmartDashboard.putData("IntakeDeploy", new IntakeDeploy(m_intake, false));
    SmartDashboard.putData("IntakeStow", new IntakeDeploy(m_intake, false));

    SmartDashboard.putData("Intake-STOP", new IntakeRun(m_intake, INMode.INTAKE_STOP));
    SmartDashboard.putData("Intake-ACQUIRE", new IntakeRun(m_intake, INMode.INTAKE_ACQUIRE));
    SmartDashboard.putData("Intake-EXPEL", new IntakeRun(m_intake, INMode.INTAKE_EXPEL));
    SmartDashboard.putData("IntakingAction", new IntakingAction(m_intake, m_floorConveyor, m_towerConveyor));
    SmartDashboard.putData("IntakingStop", new IntakingStop(m_intake, m_floorConveyor, m_towerConveyor));

    SmartDashboard.putData("LEDSet", new LEDSet(m_led, LEDColor.LEDCOLOR_OFF));
    SmartDashboard.putData("RobotInitialize", new RobotInitialize( ));

    SmartDashboard.putData("ScoringActionHighHub", new ScoringActionHighHub(0, m_shooter));
    SmartDashboard.putData("ScoringActionLowHub", new ScoringActionLowHub(0, m_shooter));
    SmartDashboard.putData("ScoringPrime", new ScoringPrime(m_shooter));
    SmartDashboard.putData("ScoringStop", new ScoringStop(m_shooter));

    SmartDashboard.putData("Shooter-OFF", new ShooterRun(m_shooter, Mode.SHOOTER_STOP));
    SmartDashboard.putData("Shooter-PRIME", new ShooterRun(m_shooter, Mode.SHOOTER_PRIME));
    SmartDashboard.putData("Shooter-LOW", new ShooterRun(m_shooter, Mode.SHOOTER_LOWERHUB));
    SmartDashboard.putData("Shooter-HIGH", new ShooterRun(m_shooter, Mode.SHOOTER_UPPERHUB));
    SmartDashboard.putData("Shooter-REV", new ShooterRun(m_shooter, Mode.SHOOTER_REVERSE));
    SmartDashboard.putData("ShooterReverse", new ShooterReverse(m_shooter));

    SmartDashboard.putData("SimulateLimelight", new SimulateLimelight( ));

    SmartDashboard.putData("Tconveyor-STOP", new TowerConveyorRun(m_towerConveyor, TCMode.TCONVEYOR_STOP));
    SmartDashboard.putData("Tconveyor-ACQUIRE", new TowerConveyorRun(m_towerConveyor, TCMode.TCONVEYOR_ACQUIRE));
    SmartDashboard.putData("Tconveyor-ACQUIRESLOW", new TowerConveyorRun(m_towerConveyor, TCMode.TCONVEYOR_ACQUIRE_SLOW));
    SmartDashboard.putData("Tconveyor-EXPEL", new TowerConveyorRun(m_towerConveyor, TCMode.TCONVEYOR_EXPEL));
    SmartDashboard.putData("Tconveyor-EXPELFAST", new TowerConveyorRun(m_towerConveyor, TCMode.TCONVEYOR_EXPEL_FAST));

    SmartDashboard.putData("Dummy", new Dummy(2135)); // TODO: Remove me when all commands/buttons are completed
  }

  private void initDefaultCommands( )
  {
    // Configure default commands for these subsystems
    m_drivetrain.setDefaultCommand(new DriveTeleop(m_drivetrain, m_driver));
    m_climber.setDefaultCommand(new ClimberRun(m_climber));
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
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    final AxisTrigger driverLeftTrigger = new AxisTrigger(m_driver, XboxController.Axis.kLeftTrigger);
    final AxisTrigger driverRightTrigger = new AxisTrigger(m_driver, XboxController.Axis.kRightTrigger);
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    // final AxisTrigger driverLeftTrigger = new AxisTrigger(m_driver, XboxController.Axis.kRightX);
    // final AxisTrigger driverRightTrigger = new AxisTrigger(m_driver, XboxController.Axis.kRightY);

    // Driver - A, B, X, Y
    driverA.whileHeld(new DriveQuickturn(m_drivetrain), true);
    driverB.whenPressed(new Dummy(XboxController.Button.kB.value), true);
    driverX.whenPressed(new Dummy(XboxController.Button.kX.value), true);
    driverY.whenPressed(new Dummy(XboxController.Button.kY.value), true);

    // Driver - Bumpers, start, back
    driverLeftBumper.whenPressed(new IntakingAction(m_intake, m_floorConveyor, m_towerConveyor), true);
    driverLeftBumper.whenReleased(new IntakingStop(m_intake, m_floorConveyor, m_towerConveyor), true);
    driverRightBumper.whenPressed(new ScoringActionLowHub(10.0, m_shooter), true);
    driverRightBumper.whenReleased(new ScoringStop(m_shooter), true);
    driverBack.whenPressed(new Dummy(XboxController.Button.kBack.value), true);
    driverStart.whenPressed(new VisionOn(m_vision, true), true);
    driverStart.whenReleased(new VisionOn(m_vision, false), true);

    // Operator - POV buttons
    driverUp.whenPressed(new Dummy(0), true);
    driverRight.whenPressed(new Dummy(90), true);
    driverDown.whenPressed(new Dummy(180), true);
    driverLeft.whenPressed(new Dummy(270), true);

    // Driver - Triggers
    driverLeftTrigger
        .whenActive(new DriveLimelightShoot(m_drivetrain, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_vision));
    driverRightTrigger.whenActive(new DriveLimelightStop(m_drivetrain));

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
    operX.whenPressed(new ScoringStop(m_shooter), true);
    operY.whenPressed(new ClimberTimerOverride(m_climber, m_operator, XboxController.Button.kY), true);

    // Operator - Bumpers, start, back
    operLeftBumper.whenPressed(new IntakingAction(m_intake, m_floorConveyor, m_towerConveyor), true);
    operLeftBumper.whenReleased(new IntakingStop(m_intake, m_floorConveyor, m_towerConveyor), true);
    operRightBumper.whenPressed(new ScoringPrime(m_shooter), true);
    operBack.whenPressed(new ClimberFullClimb(m_climber), true);
    operStart.whenPressed(new ClimberRun(m_climber), true);

    // Operator - POV buttons
    operUp.whenPressed(new Climber1Deploy(m_climber, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_drivetrain), true);
    operRight.whenPressed(new ClimberL3ToL4(m_climber, m_intake, m_floorConveyor, m_towerConveyor, m_shooter), true);
    operDown.whenPressed(new Climber0Stow(m_climber, m_intake, m_floorConveyor, m_towerConveyor, m_shooter, m_drivetrain), true);
    operLeft.whenPressed(new ClimberL2ToL3(m_climber, m_intake, m_floorConveyor, m_towerConveyor, m_shooter), true);

    // Operator Left/Right Trigger
    operLeftTrigger.whenActive(new ClimberCalibrate(m_climber), true);
    operRightTrigger.whileActiveContinuous(new ShooterReverse(m_shooter), true);
  }

  private void initAutonomousChooser( )
  {
    // Configure autonomous sendable chooser
    m_chooser.addOption("Auto1Ball1OppRight", new Auto1Ball1OppRight( ));
    m_chooser.addOption("Auto1Ball2OppLeft", new Auto1Ball2OppLeft( ));
    m_chooser.addOption("Auto1BallLimelight", new Auto1BallLimelight( ));
    m_chooser.addOption("Auto3BallLeft", new Auto3BallLeft( ));
    m_chooser.addOption("Auto3BallRight", new Auto3BallRight( ));
    m_chooser.addOption("AutoShootDriveShoot", new AutoShootDriveShoot( ));
    m_chooser.setDefaultOption("AutoStop", new AutoStop(m_drivetrain));

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
}
