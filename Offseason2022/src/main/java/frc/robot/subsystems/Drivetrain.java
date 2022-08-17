
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.frc2135.PhoenixUtil;
import frc.robot.frc2135.RobotConfig;

/**
 *
 */
/**
 *
 */
public class Drivetrain extends SubsystemBase
{
  private PigeonIMU                       m_gyro                = new PigeonIMU(0);

  private WPI_TalonFX                     m_driveL1             = new WPI_TalonFX(1);;
  private WPI_TalonFX                     m_driveL2             = new WPI_TalonFX(2);;
  private WPI_TalonFX                     m_driveR3             = new WPI_TalonFX(3);;
  private WPI_TalonFX                     m_driveR4             = new WPI_TalonFX(4);;

  DifferentialDrive                       m_diffDrive           = new DifferentialDrive(m_driveL1, m_driveR3);

  private boolean                         m_talonValidL1; // Health indicator for drive Talon Left 1
  private boolean                         m_talonValidL2; // Health indicator for drive Talon Left 2
  private boolean                         m_talonValidR3; // Health indicator for drive Talon Right 3
  private boolean                         m_talonValidR4; // Health indicator for drive Talon Right 4
  private boolean                         m_pigeonValid;  // Health indicator for Pigeon IMU

  // Declare constants
  private final int                       m_driveDebug          = 0;     // Debug flag to disable extra drive logging
                                                                         // calls
  private final int                       m_ramseteDebug        = 1;   // Debug flag to disable extra ramsete logging
                                                                       // calls
  private final int                       m_limelightDebug      = 0; // Debug flag to disable extra limelight logging
                                                                     // calls
  private final int                       kSlotIndex            = 0;       // PID slot index for sensors
  private final int                       kPidIndex             = 0;        // PID index for primary sensor
  private final int                       kCANTimeout           = 30;     // CAN timeout in msec to wait for response

  // Current limit settings
  private SupplyCurrentLimitConfiguration m_supplyCurrentLimits = new SupplyCurrentLimitConfiguration(true, 45.0, 45.0,
      0.001);
  private StatorCurrentLimitConfiguration m_statorCurrentLimits = new StatorCurrentLimitConfiguration(true, 80.0, 80.0,
      0.001);

  // Joysticks
  private double                          m_driveXScaling;  // Scaling applied to Joystick
  private double                          m_driveYScaling;  // Scaling applied to Joystick
  private double                          m_driveQTScaling; // Scaling applied to Joystick when QuickTurn enabled
  private double                          m_driveCLScaling; // Scaling applied to Joystick when slow drive mode for
  // climb enabled
  private boolean                         m_throttleZeroed;   // Throttle joystick zeroed check for safety

  // Drive modes
  boolean                                 m_brakeMode;       // Brake or Coast Mode for Talons
  boolean                                 m_isQuickTurn;     // Setting for quickturn in curvature drive
  boolean                                 m_isDriveSlowMode; // Setting for slow drive mode before climbing

  // Talon input filter settings
  private double                          m_openLoopRampRate;
  private double                          m_closedLoopRampRate;

  // Path following variables
  private double                          m_tolerance;

  // Gyro Measurements
  private double                          m_yaw;
  private double                          m_pitch;
  private double                          m_roll;

  private double                          m_gyroOffset;

  // limelight drive
  private double                          m_turnConstant        = 0;
  private double                          m_turnPidKp           = 0.1;
  private double                          m_turnPidKi           = 0.0;
  private double                          m_turnPidKd           = 0.0;
  private double                          m_throttlePidKp       = 0.1;
  private double                          m_throttlePidKi       = 0.0;
  private double                          m_throttlePidKd       = 0.0;
  private double                          m_maxTurn;
  private double                          m_maxThrottle;
  private double                          m_targetAngle;
  private double                          m_setPointDistance;
  private double                          m_angleThreshold;
  private double                          m_distThreshold;
  private double                          m_throttleShape;
  private double                          m_distOffset;
  private double                          m_limelightDistance;

  // Ramsete path follower drive
  private double                          m_ramsetePidKf        = 0.0;
  private double                          m_ramsetePidKp        = 0.0;
  private double                          m_ramsetePidKi        = 0.0;
  private double                          m_ramsetePidKd        = 0.0;
  private double                          m_ramseteB            = 0.0;
  private double                          m_ramseteZeta         = 0.0;
  private boolean                         m_ramseteTuningMode;

  /**
   *
   */
  public Drivetrain( )
  {
    setName("Drivetrain");

    setSubsystem("Drivetrain");

    addChild("Diff Drive", m_diffDrive);
    m_diffDrive.setSafetyEnabled(true);
    m_diffDrive.setExpiration(0.250);
    m_diffDrive.setMaxOutput(1.0);

    Field2d field = new Field2d( );

    // Validate Talon controllers, reset and display firmware versions

    // TODO: define globaly since the declaration is global already
    m_talonValidL1 = PhoenixUtil.getInstance( ).talonFXInitialize(m_driveL1, "L1");
    m_talonValidL2 = PhoenixUtil.getInstance( ).talonFXInitialize(m_driveL2, "L2");
    m_talonValidR3 = PhoenixUtil.getInstance( ).talonFXInitialize(m_driveR3, "R3");
    m_talonValidR4 = PhoenixUtil.getInstance( ).talonFXInitialize(m_driveR4, "R4");
    m_pigeonValid = PhoenixUtil.getInstance( ).pigeonIMUInitialize(m_gyro);

    SmartDashboard.putBoolean("HL_L1Valid", m_talonValidL1);
    SmartDashboard.putBoolean("HL_L2Valid", m_talonValidL2);
    SmartDashboard.putBoolean("HL_R3Valid", m_talonValidR3);
    SmartDashboard.putBoolean("HL_R4Valid", m_talonValidR4);

    // Load config file values
    configFileLoad( );

    // Initialize Talon motor controllers
    if (m_talonValidL1)
      talonMasterInitialize(m_driveL1, false);
    if (m_talonValidL2)
      talonFollowerInitialize(m_driveL2, 1);
    if (m_talonValidR3)
      talonMasterInitialize(m_driveR3, true);
    if (m_talonValidR4)
      talonFollowerInitialize(m_driveR4, 3);

    // If either master drive talons are valid, enable safety timer
    m_diffDrive.setSafetyEnabled(m_talonValidL1 || m_talonValidR3);

    // TODO: port rest of Constructor in
    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run when in simulation
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  void initialize( )
  {

  }

  void configFileLoad( )
  {
    // Retrieve drivetrain modified parameters from RobotConfig
    RobotConfig config = RobotConfig.getInstance( );
    m_driveXScaling = config.getValueAsDouble("DT_DriveXScaling", 0.75);
    m_driveYScaling = config.getValueAsDouble("DT_DriveYScaling", 0.75);
    m_driveQTScaling = config.getValueAsDouble("DT_QuickTurnScaling", 0.5);
    m_driveCLScaling = config.getValueAsDouble("DT_SlowClimbModeScaling", 0.3);
    m_openLoopRampRate = config.getValueAsDouble("DT_OpenLoopRampRate", 0.5);
    m_closedLoopRampRate = config.getValueAsDouble("DT_ClosedLoopRampRate", 0.0);
    m_tolerance = config.getValueAsDouble("DT_StoppedTolerance", 0.05);

    // retrieve limelight values from config file and put on smartdashboard
    m_turnConstant = config.getValueAsDouble("DTL_TurnConstant", 0);
    m_turnPidKp = config.getValueAsDouble("DTL_TurnPidKp", 0.045);
    m_turnPidKi = config.getValueAsDouble("DTL_TurnPidKi", 0.0);

    m_turnPidKd = config.getValueAsDouble("DTL_TurnPidKd", 0.0);
    m_throttlePidKp = config.getValueAsDouble("DTL_ThrottlePidKp", 0.02);
    m_throttlePidKi = config.getValueAsDouble("DTL_ThrottlePidKi", 0.0);
    m_throttlePidKd = config.getValueAsDouble("DTL_ThrottlePidKd", 0.0);

    m_maxTurn = config.getValueAsDouble("DTL_MaxTurn", 0.4);
    m_maxTurn = config.getValueAsDouble("DTL_MaxThrottle", 0.2);
    m_throttleShape = config.getValueAsDouble("DTL_ThrottleShape", 10.0);
    m_targetAngle = config.getValueAsDouble("DTL_TargetAngle", 0.0);
    m_setPointDistance = config.getValueAsDouble("DTL_SetPointDistance", 60.0);
    m_angleThreshold = config.getValueAsDouble("DTL_AngleThreshold", 3.0);
    m_distThreshold = config.getValueAsDouble("DTL_DistThreshold", 6.0);

    // Ramsete follower settings
    m_ramsetePidKf = config.getValueAsDouble("DTR_RamsetePidKf", 0.0);
    m_ramsetePidKp = config.getValueAsDouble("DTR_RamsetePidKp", 2.0);
    m_ramsetePidKi = config.getValueAsDouble("DTR_RamsetePidKi", 0.0);
    m_ramsetePidKd = config.getValueAsDouble("DTR_RamsetePidKd", 0.0);
    m_ramseteB = config.getValueAsDouble("DTR_RamseteB", 2.0);
    m_ramseteZeta = config.getValueAsDouble("DTR_RamseteZeta", 0.7);
    m_ramseteTuningMode = config.getValueAsBool("DTR_RamseteTuningMode", false);

    // Put tunable items to dashboard
    SmartDashboard.putNumber("DT_Tolerance", m_tolerance);
    SmartDashboard.putNumber("DT_GyroYaw", m_yaw);
    SmartDashboard.putNumber("DT_GyroPitch", m_pitch);
    SmartDashboard.putNumber("DT_GyroRoll", m_roll);

    SmartDashboard.putNumber("DTL_TurnConstant", m_turnConstant);
    SmartDashboard.putNumber("DTL_TurnPidKp", m_turnPidKp);
    SmartDashboard.putNumber("DTL_TurnPidKi", m_turnPidKi);
    SmartDashboard.putNumber("DTL_TurnPidKd", m_turnPidKd);
    SmartDashboard.putNumber("DTL_ThrottlePidKp", m_throttlePidKp);
    SmartDashboard.putNumber("DTL_ThrottlePidKi", m_throttlePidKi);
    SmartDashboard.putNumber("DTL_ThrottlePidKd", m_throttlePidKd);
    SmartDashboard.putNumber("DTL_MaxTurn", m_maxTurn);
    SmartDashboard.putNumber("DTL_MaxThrottle", m_maxThrottle);
    SmartDashboard.putNumber("DTL_ThrottleShape", m_throttleShape);
    SmartDashboard.putNumber("DTL_TargetAngle", m_targetAngle);
    SmartDashboard.putNumber("DTL_SetPointDistance", m_setPointDistance);
    SmartDashboard.putNumber("DTL_AngleThreshold", m_angleThreshold);
    SmartDashboard.putNumber("DTL_DistThreshold", m_distThreshold);
    SmartDashboard.putNumber("DTR_ramsetePidKf", m_ramsetePidKf);
    SmartDashboard.putNumber("DTR_ramsetePidKp", m_ramsetePidKp);
    SmartDashboard.putNumber("DTR_ramsetePidKi", m_ramsetePidKi);
    SmartDashboard.putNumber("DTR_ramsetePidKd", m_ramsetePidKd);
    SmartDashboard.putNumber("DTR_ramseteB", m_ramseteB);
    SmartDashboard.putNumber("DTR_ramseteZeta", m_ramseteZeta);
  }

  void talonMasterInitialize(WPI_TalonFX motor, boolean inverted)
  {
    motor.setInverted(inverted);
    motor.setNeutralMode(NeutralMode.Coast);

    motor.set(ControlMode.PercentOutput, 0.0);
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPidIndex, kCANTimeout);
    motor.setSensorPhase(false);
    motor.setSelectedSensorPosition(0, kPidIndex, kCANTimeout);

    PhoenixUtil.getInstance( ).checkError(motor.configOpenloopRamp(m_openLoopRampRate, kCANTimeout),
        "HL_ConfigOpenloopRamp");
    PhoenixUtil.getInstance( ).checkError(
        motor.configClosedloopRamp(m_closedLoopRampRate, kCANTimeout),
        "HL_ConfigClosedloopRamp");
    PhoenixUtil.getInstance( ).checkError(
        motor.configSupplyCurrentLimit(m_supplyCurrentLimits),
        "HL_ConfigSupplyCurrentLimit");
    PhoenixUtil.getInstance( ).checkError(motor.configStatorCurrentLimit(m_statorCurrentLimits),
        "HL_ConfigStatorCurrentLimit");
  }

  void talonFollowerInitialize(WPI_TalonFX motor, int master)
  {
    motor.set(ControlMode.Follower, master);
    motor.setInverted(InvertType.FollowMaster);
    motor.setNeutralMode(NeutralMode.Coast);
    PhoenixUtil.getInstance( ).checkError(
        motor.setStatusFramePeriod(StatusFrame.Status_1_General, 255, kCANTimeout),
        "HL_SetStatusFramePeriod_Status1");
    PhoenixUtil.getInstance( ).checkError(
        motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255, kCANTimeout),
        "HL_SetStatusFramePeriod_Status2");
    PhoenixUtil.getInstance( ).checkError(
        motor.configSupplyCurrentLimit(m_supplyCurrentLimits),
        "HL_ConfigSupplyCurrentLimit");
    PhoenixUtil.getInstance( ).checkError(
        motor.configStatorCurrentLimit(m_statorCurrentLimits),
        "HL_ConfigStatorCurrentLimit");
  }

  void setBrakeMode(boolean brakeMode)
  {
    m_brakeMode = brakeMode;

    if (brakeMode)
    {
      DataLogManager.log(getSubsystem( ) + ": Mode BRAKE");
    }
    else
    {
      DataLogManager.log(getSubsystem( ) + ": Mode COAST");
    }
    SmartDashboard.putBoolean("DT_BrakeMode", brakeMode);

    NeutralMode brakeOutput;
    brakeOutput = (brakeMode) ? NeutralMode.Brake : NeutralMode.Coast;
    if (m_talonValidL1)
      m_driveL1.setNeutralMode(brakeOutput);
    if (m_talonValidL2)
      m_driveL2.setNeutralMode(brakeOutput);
    if (m_talonValidR3)
      m_driveR3.setNeutralMode(brakeOutput);
    if (m_talonValidR4)
      m_driveR4.setNeutralMode(brakeOutput);
  }

  //
  // Set quick turn for curvature drive
  //
  void moveSetQuickTurn(boolean quickTurn)
  {
    m_isQuickTurn = quickTurn;
  }

  void moveWithJoysticksInit( )
  {
    setBrakeMode(true);
    m_driveL1.configOpenloopRamp(m_openLoopRampRate);
    m_driveR3.configOpenloopRamp(m_openLoopRampRate);
  }

  void moveWithJoysticks(XboxController throttleJstick)
  {
    double xValue = throttleJstick.getRightX( );
    double yValue = throttleJstick.getLeftY( );
    double xOutput = 0.0;
    double yOutput = 0.0;

    // If joysticks report a very small value, then stick has been centered
    if (Math.abs(yValue) < 0.05 && Math.abs(xValue) < 0.05)
    {
      m_throttleZeroed = true;
    }

    // If throttle and steering not centered, use zero outputs until they do
    if (m_throttleZeroed)
    {
      if (m_isQuickTurn)
      {
        xOutput = m_driveQTScaling * (xValue * Math.abs(xValue));
        yOutput = m_driveQTScaling * (yValue * Math.abs(yValue));
      }
      else if (m_isDriveSlowMode)
      {
        xOutput = m_driveCLScaling * (xValue * Math.abs(xValue));
        yOutput = m_driveCLScaling * (yValue * Math.abs(yValue));
      }
      else
      {
        xOutput = m_driveXScaling * (xValue * Math.abs(xValue));
        yOutput = m_driveYScaling * (yValue * Math.abs(yValue));
      }
    }

    if (m_talonValidL1 || m_talonValidR3)
      m_diffDrive.curvatureDrive(yOutput, xOutput, m_isQuickTurn);
  }

  void MoveWithJoysticksEnd( )
  {
    setBrakeMode(false);
    m_driveL1.configOpenloopRamp(0.0);
    m_driveR3.configOpenloopRamp(0.0);
  }

}
