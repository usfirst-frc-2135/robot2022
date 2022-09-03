
// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.FCConsts.FCMode;
import frc.robot.Constants.INConsts.INMode;
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
import frc.robot.commands.AutoStop;
import frc.robot.commands.AutonomousCommand;
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
import frc.robot.commands.ClimberMoveToHeight;
import frc.robot.commands.ClimberRun;
import frc.robot.commands.ClimberSetGatehook;
import frc.robot.commands.ClimberTimerOverride;
import frc.robot.commands.DriveLimelight;
import frc.robot.commands.DriveMotorTest;
import frc.robot.commands.DriveQuickturn;
import frc.robot.commands.DriveResetSensors;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.Dummy;
import frc.robot.commands.ExhaustingAction;
import frc.robot.commands.ExhaustingStop;
import frc.robot.commands.FloorConveyorRun;
import frc.robot.commands.IntakeDeploy;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.IntakingAction;
import frc.robot.commands.IntakingStop;
import frc.robot.commands.RobotInitialize;
import frc.robot.commands.ScoringActionLowHub;
import frc.robot.commands.ScoringPrime;
import frc.robot.commands.ScoringStop;
import frc.robot.commands.VisionOn;
import frc.robot.commands.ShooterAimToggle;
import frc.robot.commands.ShooterReverse;
import frc.robot.commands.ShooterRun;
import frc.robot.commands.SimulateLimelight;
import frc.robot.commands.TowerConveyorRun;
import frc.robot.frc2135.RobotConfig;
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
  private static RobotConfig    m_robotConfig    = RobotConfig.getInstance( );

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
  private final XboxController  driver           = new XboxController(0);
  private final XboxController  operator         = new XboxController(1);

  // A chooser for autonomous commands
  SendableChooser<Command>      m_chooser        = new SendableChooser<>( );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer( )
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
    SmartDashboard.putData("DriveMotorTest", new DriveMotorTest(m_drivetrain, true));
    SmartDashboard.putData("DriveResetSensors", new DriveResetSensors(m_drivetrain));
    SmartDashboard.putData("ExhaustingAction", new ExhaustingAction( ));
    SmartDashboard.putData("ExhaustingStop", new ExhaustingStop( ));
    SmartDashboard.putData("Fconveyor-STOP", new FloorConveyorRun(FCMode.FCONVEYOR_STOP, m_floorConveyor));
    SmartDashboard.putData("Fconveyor-ACQUIRE", new FloorConveyorRun(FCMode.FCONVEYOR_ACQUIRE, m_floorConveyor));
    SmartDashboard.putData("Fconveyor-EXPEL", new FloorConveyorRun(FCMode.FCONVEYOR_EXPEL, m_floorConveyor));
    SmartDashboard.putData("Fconveyor-EXPELFAST", new FloorConveyorRun(FCMode.FCONVEYOR_EXPEL_FAST, m_floorConveyor));
    SmartDashboard.putData("IntakingAction", new IntakingAction( ));
    SmartDashboard.putData("IntakingStop", new IntakingStop( ));
    SmartDashboard.putData("Intake-STOP", new IntakeRun(m_intake, INMode.INTAKE_STOP));
    SmartDashboard.putData("Intake-ACQUIRE", new IntakeRun(m_intake, INMode.INTAKE_ACQUIRE));
    SmartDashboard.putData("Intake-EXPEL", new IntakeRun(m_intake, INMode.INTAKE_EXPEL));
    SmartDashboard.putData("RobotInitialize", new RobotInitialize( ));
    SmartDashboard.putData("ShooterAimToggle", new ShooterAimToggle( ));
    SmartDashboard.putData("Shooter-OFF", new ShooterRun(Mode.SHOOTER_STOP, m_shooter));
    SmartDashboard.putData("Shooter-PRIME", new ShooterRun(Mode.SHOOTER_PRIME, m_shooter));
    SmartDashboard.putData("Shooter-LOW", new ShooterRun(Mode.SHOOTER_LOWERHUB, m_shooter));
    SmartDashboard.putData("Shooter-HIGH", new ShooterRun(Mode.SHOOTER_UPPERHUB, m_shooter));
    SmartDashboard.putData("Shooter-REV", new ShooterRun(Mode.SHOOTER_REVERSE, m_shooter));
    SmartDashboard.putData("ShootReverse", new ShooterReverse(m_shooter));
    SmartDashboard.putData("SimulateLimelight", new SimulateLimelight( ));
    SmartDashboard.putData("Tconveyor-STOP", new TowerConveyorRun(m_towerConveyor, TCMode.TCONVEYOR_STOP));
    SmartDashboard.putData("Tconveyor-ACQUIRE", new TowerConveyorRun(m_towerConveyor, TCMode.TCONVEYOR_ACQUIRE));
    SmartDashboard.putData("Tconveyor-ACQUIRESLOW", new TowerConveyorRun(m_towerConveyor, TCMode.TCONVEYOR_ACQUIRE_SLOW));
    SmartDashboard.putData("Tconveyor-EXPEL", new TowerConveyorRun(m_towerConveyor, TCMode.TCONVEYOR_EXPEL));
    SmartDashboard.putData("Tconveyor-EXPELFAST", new TowerConveyorRun(m_towerConveyor, TCMode.TCONVEYOR_EXPEL_FAST));
    SmartDashboard.putData("Dummy", new Dummy( ));

    // Configure the button bindings
    configureButtonBindings( );

    // Configure default commands
    m_drivetrain.setDefaultCommand(new DriveTeleop(m_drivetrain));
    m_climber.setDefaultCommand(new ClimberRun(m_climber));

    // Configure autonomous sendable chooser

    m_chooser.addOption("Autonomous Command", new AutonomousCommand( ));
    m_chooser.addOption("Auto1Ball1OppRight", new Auto1Ball1OppRight( ));
    m_chooser.addOption("Auto1Ball2OppLeft", new Auto1Ball2OppLeft( ));
    m_chooser.addOption("Auto1BallLimelight", new Auto1BallLimelight( ));
    m_chooser.addOption("Auto3BallLeft", new Auto3BallLeft( ));
    m_chooser.addOption("Auto3BallRight", new Auto3BallRight( ));
    m_chooser.addOption("AutoShootDriveShoot", new AutoShootDriveShoot( ));
    m_chooser.addOption("AutoStop", new AutoStop(m_drivetrain));
    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand( ));

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance( )
  {
    return m_robotContainer;
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
    final JoystickButton operBack = new JoystickButton(operator, XboxController.Button.kBack.value);
    operBack.whenPressed(new ClimberFullClimb( ), true);

    final JoystickButton operStart = new JoystickButton(operator, XboxController.Button.kStart.value);
    operStart.whenPressed(new ClimberRun(m_climber), true);

    final JoystickButton operRightBumper = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    operRightBumper.whenPressed(new ScoringPrime(0, m_shooter), true);

    final JoystickButton operLeftBumper = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    operLeftBumper.whenPressed(new IntakingAction( ), true);

    final JoystickButton operY = new JoystickButton(operator, XboxController.Button.kY.value);
    operY.whenPressed(new Dummy( ), true);

    final JoystickButton operX = new JoystickButton(operator, XboxController.Button.kX.value);
    operX.whenPressed(new ScoringStop(m_shooter), true);

    final JoystickButton operB = new JoystickButton(operator, XboxController.Button.kB.value);
    operB.whenPressed(new ExhaustingAction( ), true);

    final JoystickButton operA = new JoystickButton(operator, XboxController.Button.kA.value);
    operA.whenPressed(new IntakeDeploy(false), true);

    final JoystickButton driverBack = new JoystickButton(driver, XboxController.Button.kBack.value);
    driverBack.whenPressed(new Dummy( ), true);

    final JoystickButton driverStart = new JoystickButton(driver, XboxController.Button.kStart.value);
    driverStart.whenPressed(new VisionOn(false), true);

    final JoystickButton driverRightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    driverRightBumper.whenPressed(new ScoringActionLowHub(0, m_shooter), true);

    final JoystickButton driverLeftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    driverLeftBumper.whenPressed(new IntakingAction( ), true);

    final JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
    driverY.whenPressed(new Dummy( ), true);

    final JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
    driverX.whenPressed(new Dummy( ), true);

    final JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
    driverB.whenPressed(new Dummy( ), true);

    final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    driverA.whenPressed(new DriveQuickturn( ), true);
  }

  public XboxController getDriver( )
  {
    return driver;
  }

  public XboxController getOperator( )
  {
    return operator;
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
