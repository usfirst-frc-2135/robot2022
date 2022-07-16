
// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

    // The robot's subsystems
  public final Vision m_vision = new Vision();
  public final LED m_lED = new LED();
  public final Power m_power = new Power();
  public final Pneumatics m_pneumatics = new Pneumatics();
  public final Climber m_climber = new Climber();
  public final Shooter m_shooter = new Shooter();
  public final TowerConveyor m_towerConveyor = new TowerConveyor();
  public final FloorConveyor m_floorConveyor = new FloorConveyor();
  public final Intake m_intake = new Intake();
  public final Drivetrain m_drivetrain = new Drivetrain();

  // Joysticks
  private final XboxController operator = new XboxController(1);
  private final XboxController driver = new XboxController(0);

  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and
   * commands.
   */
  private RobotContainer() {
        // Smartdashboard Subsystems

    // SmartDashboard Buttons
    SmartDashboard.putData("Auto1Ball1OppRight", new Auto1Ball1OppRight());
    SmartDashboard.putData("Auto1Ball2OppLeft", new Auto1Ball2OppLeft());
    SmartDashboard.putData("Auto1BallLimelight", new Auto1BallLimelight());
    SmartDashboard.putData("Auto3BallLeft", new Auto3BallLeft());
    SmartDashboard.putData("Auto3BallRight", new Auto3BallRight());
    SmartDashboard.putData("AutoDrive", new AutoDrive());
    SmartDashboard.putData("AutoDriveLimelightShoot",
                           new AutoDriveLimelightShoot());
    SmartDashboard.putData("AutoDrivePath", new AutoDrivePath());
    SmartDashboard.putData("AutoDriveShoot", new AutoDriveShoot());
    SmartDashboard.putData("AutoPathSequence", new AutoPathSequence());
    SmartDashboard.putData("AutoShoot", new AutoShoot());
    SmartDashboard.putData("AutoShootDriveShoot", new AutoShootDriveShoot());
    SmartDashboard.putData("AutoStop", new AutoStop(m_drivetrain));
    SmartDashboard.putData("Climber0Stow", new Climber0Stow());
    SmartDashboard.putData("Climber1Deploy", new Climber1Deploy());
    SmartDashboard.putData("Climber2ClimbToL2", new Climber2ClimbToL2());
    SmartDashboard.putData("Climber3RotateToL3", new Climber3RotateToL3());
    SmartDashboard.putData("Climber5RotateIntoL3", new Climber5RotateIntoL3());
    SmartDashboard.putData("Climber6ClimbToL3", new Climber6ClimbToL3());
    SmartDashboard.putData("Climber7ClimbToL4", new Climber7ClimbToL4());
    SmartDashboard.putData("Climber8SettleToL4", new Climber8SettleToL4());
    SmartDashboard.putData("ClimberCalibrate", new ClimberCalibrate(m_climber));
    SmartDashboard.putData("ClimberFullClimb", new ClimberFullClimb());
    SmartDashboard.putData("ClimberL2ToL3", new ClimberL2ToL3());
    SmartDashboard.putData("ClimberL3ToL4", new ClimberL3ToL4());
    SmartDashboard.putData("ClimberMoveToHeight",
                           new ClimberMoveToHeight(m_climber));
    SmartDashboard.putData("ClimberSetGatehook", new ClimberSetGatehook());
    SmartDashboard.putData("ClimberTimerOverride", new ClimberTimerOverride());
    SmartDashboard.putData("DriveLimelight", new DriveLimelight());
    SmartDashboard.putData("DriveMotorTest", new DriveMotorTest());
    SmartDashboard.putData("DriveResetSensors", new DriveResetSensors());
    SmartDashboard.putData("ExhaustingAction", new ExhaustingAction());
    SmartDashboard.putData("ExhaustingStop", new ExhaustingStop());
    SmartDashboard.putData("IntakingAction", new IntakingAction());
    SmartDashboard.putData("IntakingStop", new IntakingStop());
    SmartDashboard.putData("RobotInitialize", new RobotInitialize());
    SmartDashboard.putData("ShooterAimToggle", new ShooterAimToggle());
    SmartDashboard.putData("ShooterReverse", new ShooterReverse());
    SmartDashboard.putData("SimulateLimelight", new SimulateLimelight());
    SmartDashboard.putData("Dummy", new Dummy());

        // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
        // ID=SUBSYSTEM_DEFAULT_COMMAND
    m_climber.setDefaultCommand(new ClimberRun(m_climber));
    m_shooter.setDefaultCommand(new ShooterRun(0, m_shooter));
    m_towerConveyor.setDefaultCommand(new TowerConveyorRun(0));
    m_floorConveyor.setDefaultCommand(new IntakeRun(0, m_intake));
    m_intake.setDefaultCommand(new IntakeRun(0, m_intake));
    m_drivetrain.setDefaultCommand(new DriveTeleop(m_drivetrain));

    
    // Configure autonomous sendable chooser
    
    m_chooser.addOption("Autonomous Command", new AutonomousCommand());
    m_chooser.addOption("Auto1Ball1OppRight", new Auto1Ball1OppRight());
    m_chooser.addOption("Auto1Ball2OppLeft", new Auto1Ball2OppLeft());
    m_chooser.addOption("Auto1BallLimelight", new Auto1BallLimelight());
    m_chooser.addOption("Auto3BallLeft", new Auto3BallLeft());
    m_chooser.addOption("Auto3BallRight", new Auto3BallRight());
    m_chooser.addOption("AutoShootDriveShoot", new AutoShootDriveShoot());
    m_chooser.addOption("AutoStop", new AutoStop(m_drivetrain));
    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    
    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() { return m_robotContainer; }

  /**
   * Use this method to define your button->command mappings.  Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and
   * then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
        // Create some buttons
    final JoystickButton operBack =
        new JoystickButton(operator, XboxController.Button.kBack.value);
    operBack.whenPressed(new ClimberFullClimb(), true);

    final JoystickButton operStart =
        new JoystickButton(operator, XboxController.Button.kStart.value);
    operStart.whenPressed(new ClimberRun(m_climber), true);

    final JoystickButton operRightBumper =
        new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    operRightBumper.whenPressed(new ScoringPrime(0, m_shooter), true);

    final JoystickButton operLeftBumper =
        new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    operLeftBumper.whenPressed(new IntakingAction(), true);

    final JoystickButton operY =
        new JoystickButton(operator, XboxController.Button.kY.value);
    operY.whenPressed(new Dummy(), true);

    final JoystickButton operX =
        new JoystickButton(operator, XboxController.Button.kX.value);
    operX.whenPressed(new ScoringStop(m_shooter), true);

    final JoystickButton operB =
        new JoystickButton(operator, XboxController.Button.kB.value);
    operB.whenPressed(new ExhaustingAction(), true);

    final JoystickButton operA =
        new JoystickButton(operator, XboxController.Button.kA.value);
    operA.whenPressed(new IntakeDeploy(false), true);

    final JoystickButton driverBack =
        new JoystickButton(driver, XboxController.Button.kBack.value);
    driverBack.whenPressed(new Dummy(), true);

    final JoystickButton driverStart =
        new JoystickButton(driver, XboxController.Button.kStart.value);
    driverStart.whenPressed(new ShooterAim(false), true);

    final JoystickButton driverRightBumper =
        new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    driverRightBumper.whenPressed(new ScoringActionLowHub(0, m_shooter), true);

    final JoystickButton driverLeftBumper =
        new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    driverLeftBumper.whenPressed(new IntakingAction(), true);

    final JoystickButton driverY =
        new JoystickButton(driver, XboxController.Button.kY.value);
    driverY.whenPressed(new Dummy(), true);

    final JoystickButton driverX =
        new JoystickButton(driver, XboxController.Button.kX.value);
    driverX.whenPressed(new Dummy(), true);

    final JoystickButton driverB =
        new JoystickButton(driver, XboxController.Button.kB.value);
    driverB.whenPressed(new Dummy(), true);

    final JoystickButton driverA =
        new JoystickButton(driver, XboxController.Button.kA.value);
    driverA.whenPressed(new DriveQuickturn(), true);

      }

    public XboxController getDriver() { return driver; }

  public XboxController getOperator() { return operator; }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }
}
