
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DTConsts;
import frc.robot.Constants.Falcon500;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.frc2135.PhoenixUtil;

/**
 *
 */
public class Drivetrain extends SubsystemBase
{
  // Constants
  private static final int                  CANTIMEOUT            = 30;  // CAN timeout in msec
  private static final int                  PIDINDEX              = 0;   // PID in use (0-primary, 1-aux)
  private static final int                  SLOTINDEX             = 0;   // Use first PID slot

  // Devices and simulation objects
  private final WPI_TalonFX                 m_driveL1             = new WPI_TalonFX(1);;
  private final WPI_TalonFX                 m_driveL2             = new WPI_TalonFX(2);;
  private final WPI_TalonFX                 m_driveR3             = new WPI_TalonFX(3);;
  private final WPI_TalonFX                 m_driveR4             = new WPI_TalonFX(4);;
  private final DifferentialDrive           m_diffDrive           = new DifferentialDrive(m_driveL1, m_driveR3);
  private final PigeonIMU                   m_gyro                = new PigeonIMU(0);

  private final DifferentialDriveKinematics m_kinematics          = new DifferentialDriveKinematics(DTConsts.kTrackWidthMeters);
  private final TalonFXSimCollection        m_leftSim             = m_driveL1.getSimCollection( );
  private final TalonFXSimCollection        m_rightSim            = m_driveR3.getSimCollection( );
  private final DifferentialDrivetrainSim   m_driveSim            = new DifferentialDrivetrainSim(
      LinearSystemId.identifyDrivetrainSystem(DTConsts.kv, DTConsts.ka, DTConsts.KvAngular, DTConsts.KaAngular,
          DTConsts.kTrackWidthMeters),
      DCMotor.getFalcon500(2), DTConsts.kGearRatio, DTConsts.kTrackWidthMeters, DTConsts.kWheelDiaMeters / 2, null);
  // Change parameter from "null": noise VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)

  private final BasePigeonSimCollection     m_pidgeonSim          = new BasePigeonSimCollection(m_gyro, false);

  // Current limit settings
  private SupplyCurrentLimitConfiguration   m_supplyCurrentLimits = new SupplyCurrentLimitConfiguration(true, 45.0, 45.0, 0.001);
  private StatorCurrentLimitConfiguration   m_statorCurrentLimits = new StatorCurrentLimitConfiguration(true, 80.0, 80.0, 0.001);

  // Declare module variables
  private double                            m_driveXScaling       = DTConsts.kDriveXScaling;    // X axis joystick scaling
  private double                            m_driveYScaling       = DTConsts.kDriveYScaling;    // Y axis joystick scaling
  private double                            m_driveQTScaling      = DTConsts.kQuickTurnScaling; // Quickturn joystick scaling
  private double                            m_driveCLScaling      = DTConsts.kSlowClimbScaling; // Slow speed joystick scaling

  private double                            m_openLoopRamp        = DTConsts.kOpenLoopRamp;     // Rate during teleop (opem loop)
  private double                            m_closedLoopRamp      = DTConsts.kClosedLoopRamp;   // Rate during closed loop modes
  private double                            m_stopTolerance       = DTConsts.kStopTolerance;    // Position tolerance (<5cm)

  // Limelight drive
  private double                            m_turnConstant        = DTConsts.kTurnConstant;
  private double                            m_turnPidKp           = DTConsts.kTurnPidKp;
  private double                            m_turnPidKi           = DTConsts.kTurnPidKi;
  private double                            m_turnPidKd           = DTConsts.kTurnPidKd;
  private double                            m_turnMax             = DTConsts.kTurnMax;
  private double                            m_throttlePidKp       = DTConsts.kThrottlePidKp;
  private double                            m_throttlePidKi       = DTConsts.kThrottlePidKi;
  private double                            m_throttlePidKd       = DTConsts.kThrottlePidKd;
  private double                            m_throttleMax         = DTConsts.kThrottleMax;
  private double                            m_throttleShape       = DTConsts.kThrottleShape;

  private double                            m_targetAngle         = DTConsts.kTargetAngle;      // Optimal shooting angle
  private double                            m_setPointDistance    = DTConsts.kSetPointDistance; // Optimal shooting distance
  private double                            m_angleThreshold      = DTConsts.kAngleThreshold;   // Tolerance around optimal
  private double                            m_distThreshold       = DTConsts.kDistThreshold;    // Tolerance around optimal

  // Path follower controls
  private double                            m_ramsetePidKf        = DTConsts.kRamsetePidKf;
  private double                            m_ramsetePidKp        = DTConsts.kRamsetePidKp;
  private double                            m_ramsetePidKi        = DTConsts.kRamsetePidKi;
  private double                            m_ramsetePidKd        = DTConsts.kRamsetePidKd;
  private double                            m_ramseteB            = DTConsts.kRamseteB;
  private double                            m_ramseteZeta         = DTConsts.kRamseteZeta;

  // Health variables
  private boolean                           m_validL1;                    // Health indicator for drive Talon Left 1
  private boolean                           m_validL2;                    // Health indicator for drive Talon Left 2
  private boolean                           m_validR3;                    // Health indicator for drive Talon Right 3
  private boolean                           m_validR4;                    // Health indicator for drive Talon Right 4
  private boolean                           m_pigeonValid;                // Health indicator for Pigeon IMU
  private int                               m_resetCountL1        = 0;    // motor reset count storer
  private int                               m_resetCountL2        = 0;    // motor reset count storer
  private int                               m_resetCountR3        = 0;    // motor reset count storer
  private int                               m_resetCountR4        = 0;    // motor reset count storer

  private int                               m_driveDebug          = 0;    // Debug flag to disable extra drive logging calls
  private int                               m_limelightDebug      = 0;    // Debug flag to disable extra limelight logging calls
  private int                               m_ramseteDebug        = 0;    // Debug flag to disable extra ramsete logging calls

  private boolean                           m_throttleZeroed      = false; // Throttle joystick zeroed safety check
  private boolean                           m_isQuickTurn         = false; // Quickturn mode active in curvature drive
  private boolean                           m_driveSlowMode       = false; // Slow drive mode active when climbing
  private double                            m_limelightDistance;

  private int                               m_periodicInterval    = 0;

  // Odometry and telemetry
  private double                            m_distanceLeft;              // Left wheel distance in meters
  private double                            m_distanceRight;             // Right wheel distance in meters
  private DifferentialDriveWheelSpeeds      m_wheelSpeeds;               // Wheel speeds in meters / sec
  private DifferentialDriveOdometry         m_odometry            = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0.0));
  private Field2d                           m_field               = new Field2d( );

  // Gyro Measurements
  private double                            m_gyroOffset          = 0.0; // Gyro (IMU) correction when reset
  private double                            m_yaw;                       // Gyro yaw about Z axis
  private double                            m_pitch;                     // Gyro pitch about X axis
  private double                            m_roll;                      // Gyro roll about Y axis

  // DriveWithLimelight pid controller objects
  private PIDController                     m_turnPid             = new PIDController(0.0, 0.0, 0.0);
  private PIDController                     m_throttlePid         = new PIDController(0.0, 0.0, 0.0);

  // Ramsete follower objects
  private RamseteController                 m_ramseteController;
  private Trajectory                        m_trajectory;
  private Timer                             m_trajTimer           = new Timer( );

  private double                            m_currentl1           = 0.0; // Motor L1 Falcon output current
  private double                            m_currentL2           = 0.0; // Motor L2 Falcon output current
  private double                            m_currentR3           = 0.0; // Motor R3 Falcon output current
  private double                            m_currentR4           = 0.0; // Motor R4 Falcon output current

  /**
   *
   */
  public Drivetrain( )
  {
    setName("Drivetrain");
    setSubsystem("Drivetrain");
    addChild("DiffDrive", m_diffDrive);

    m_diffDrive.setSafetyEnabled(true);
    m_diffDrive.setExpiration(0.250);
    m_diffDrive.setMaxOutput(1.0);

    // Validate Talon controllers, reset and display firmware versions
    m_validL1 = PhoenixUtil.getInstance( ).talonFXInitialize(m_driveL1, "L1");
    m_validL2 = PhoenixUtil.getInstance( ).talonFXInitialize(m_driveL2, "L2");
    m_validR3 = PhoenixUtil.getInstance( ).talonFXInitialize(m_driveR3, "R3");
    m_validR4 = PhoenixUtil.getInstance( ).talonFXInitialize(m_driveR4, "R4");
    m_pigeonValid = PhoenixUtil.getInstance( ).pigeonIMUInitialize(m_gyro);

    initSmartDashboard( );

    // Initialize Talon motor controllers
    if (m_validL1)
      initTalonMaster(m_driveL1, false);
    if (m_validL2)
      initTalonFollower(m_driveL2, DTConsts.kL1CANID);
    if (m_validR3)
      initTalonMaster(m_driveR3, true);
    if (m_validR4)
      initTalonFollower(m_driveR4, DTConsts.kR3CANID);

    // If either master drive talons are valid, enable safety timer
    m_diffDrive.setSafetyEnabled(m_validL1 || m_validR3);

    // Limelight drive Pid Controllers
    m_turnPid.setPID(m_turnPidKp, m_turnPidKi, m_turnPidKd);
    m_throttlePid.setPID(m_throttlePidKp, m_throttlePidKi, m_throttlePidKd);

    m_ramseteController = new RamseteController(m_ramseteB, m_ramseteZeta);

    syncFollowerPIDFromDashboard( );
    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
    updateOdometry( );
    updateDashboardValues( );
    m_field.setRobotPose(m_odometry.getPoseMeters( ));

    if (m_validL1 && m_driveL1.hasResetOccurred( ))
      SmartDashboard.putNumber("HL_resetCountL1", ++m_resetCountL1);
    if (m_validL2 && m_driveL2.hasResetOccurred( ))
      SmartDashboard.putNumber("HL_resetCountL2", ++m_resetCountL2);
    if (m_validR3 && m_driveR3.hasResetOccurred( ))
      SmartDashboard.putNumber("HL_resetCountR3", ++m_resetCountR3);
    if (m_validR4 && m_driveR4.hasResetOccurred( ))
      SmartDashboard.putNumber("HL_resetCountR4", ++m_resetCountR4);

    if (RobotState.isDisabled( ))
      resetGyro( );
  }

  @Override
  public void simulationPeriodic( )
  {
    /* Pass the robot battery voltage to the simulated Talon FXs */
    m_leftSim.setBusVoltage(RobotController.getInputVoltage( ));
    m_rightSim.setBusVoltage(RobotController.getInputVoltage( ));
    m_driveSim.setInputs(m_leftSim.getMotorOutputLeadVoltage( ), -m_rightSim.getMotorOutputLeadVoltage( ));

    /* Advance the model by 20 ms. */
    m_driveSim.update(0.02);

    /*
     * WPILib's sim class assumes +V is forward, but -V is forward for the right motor, negate the
     * reported position. Basically, we negated the input, so we need to negate the output.
     */
    m_leftSim.setIntegratedSensorRawPosition(metersToNativeUnits(m_driveSim.getLeftPositionMeters( )));
    m_leftSim.setIntegratedSensorVelocity(mpsToNativeUnits(m_driveSim.getLeftVelocityMetersPerSecond( )));
    m_rightSim.setIntegratedSensorRawPosition(metersToNativeUnits(-m_driveSim.getRightPositionMeters( )));
    m_rightSim.setIntegratedSensorVelocity(mpsToNativeUnits(-m_driveSim.getRightVelocityMetersPerSecond( )));
    m_pidgeonSim.setRawHeading(m_driveSim.getHeading( ).getDegrees( ));
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  public void initialize( )
  {
    DataLogManager.log(getSubsystem( ) + ": subsystem initialized!");

    m_throttleZeroed = false;
    setBrakeMode(false);
    driveSetQuickTurn(false);
    setDriveSlowMode(false);

    driveStopMotors( );

    resetGyro( );
    resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    m_driveSim.setPose(getPose( ));
  }

  public void faultDump( )
  {
    PhoenixUtil.getInstance( ).talonFXFaultDump(m_driveL1, "DT L1");
    PhoenixUtil.getInstance( ).talonFXFaultDump(m_driveL2, "DT L2");
    PhoenixUtil.getInstance( ).talonFXFaultDump(m_driveR3, "DT R3");
    PhoenixUtil.getInstance( ).talonFXFaultDump(m_driveR4, "DT R4");
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Initialization helper methods
  //
  private void initSmartDashboard( )
  {
    // Put subsystem health indicators
    SmartDashboard.putBoolean("HL_validL1", m_validL1);
    SmartDashboard.putBoolean("HL_validL2", m_validL2);
    SmartDashboard.putBoolean("HL_validR3", m_validR3);
    SmartDashboard.putBoolean("HL_validR4", m_validR4);
    SmartDashboard.putBoolean("HL_pigeonValid", m_pigeonValid);
    SmartDashboard.putNumber("HL_resetCountL1", m_resetCountL1);
    SmartDashboard.putNumber("HL_resetCountL2", m_resetCountL2);
    SmartDashboard.putNumber("HL_resetCountR3", m_resetCountR3);
    SmartDashboard.putNumber("HL_resetCountR4", m_resetCountR4);

    SmartDashboard.putNumber("DT_stopTolerance", m_stopTolerance);

    // Put tunable items to dashboard
    SmartDashboard.putNumber("DTL_turnConstant", m_turnConstant);
    SmartDashboard.putNumber("DTL_turnPidKp", m_turnPidKp);
    SmartDashboard.putNumber("DTL_turnPidKi", m_turnPidKi);
    SmartDashboard.putNumber("DTL_turnPidKd", m_turnPidKd);
    SmartDashboard.putNumber("DTL_turnMax", m_turnMax);
    SmartDashboard.putNumber("DTL_throttlePidKp", m_throttlePidKp);
    SmartDashboard.putNumber("DTL_throttlePidKi", m_throttlePidKi);
    SmartDashboard.putNumber("DTL_throttlePidKd", m_throttlePidKd);
    SmartDashboard.putNumber("DTL_throttleMax", m_throttleMax);
    SmartDashboard.putNumber("DTL_throttleShape", m_throttleShape);

    SmartDashboard.putNumber("DTL_targetAngle", m_targetAngle);
    SmartDashboard.putNumber("DTL_setPointDistance", m_setPointDistance);
    SmartDashboard.putNumber("DTL_angleThreshold", m_angleThreshold);
    SmartDashboard.putNumber("DTL_distThreshold", m_distThreshold);

    SmartDashboard.putNumber("DTR_ramsetePidKf", m_ramsetePidKf);
    SmartDashboard.putNumber("DTR_ramsetePidKp", m_ramsetePidKp);
    SmartDashboard.putNumber("DTR_ramsetePidKi", m_ramsetePidKi);
    SmartDashboard.putNumber("DTR_ramsetePidKd", m_ramsetePidKd);

    SmartDashboard.putNumber("DTR_ramseteB", m_ramseteB);
    SmartDashboard.putNumber("DTR_ramseteZeta", m_ramseteZeta);

    SmartDashboard.putData("Field", m_field);
  }

  private void initTalonMaster(WPI_TalonFX motor, boolean inverted)
  {
    motor.setInverted(inverted);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setInverted");
    motor.setNeutralMode(NeutralMode.Coast);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setNeutralMode");

    motor.set(ControlMode.PercentOutput, 0.0);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "set");
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PIDINDEX, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configSelectedFeedbackSensor");
    motor.setSensorPhase(false);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setSensorPhase");
    motor.setSelectedSensorPosition(0, PIDINDEX, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setSelectedSensorPosition");

    motor.configOpenloopRamp(m_openLoopRamp, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configOpenloopRamp");
    motor.configClosedloopRamp(m_closedLoopRamp, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configClosedloopRamp");

    motor.configSupplyCurrentLimit(m_supplyCurrentLimits);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configSupplyCurrentLimit");
    motor.configStatorCurrentLimit(m_statorCurrentLimits);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configStatorCurrentLimit");
  }

  private void initTalonFollower(WPI_TalonFX motor, int master)
  {
    motor.set(ControlMode.Follower, master);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "set");

    motor.setInverted(InvertType.FollowMaster);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setInverted");
    motor.setNeutralMode(NeutralMode.Coast);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setNeutralMode");

    motor.setStatusFramePeriod(StatusFrame.Status_1_General, 255, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setStatusFramePeriod_1");
    motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setStatusFramePeriod_2");

    motor.configSupplyCurrentLimit(m_supplyCurrentLimits);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configSupplyCurrentLimit");
    motor.configStatorCurrentLimit(m_statorCurrentLimits);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configStatorCurrentLimit");
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Periodic helper methods
  //
  private void updateOdometry( )
  {
    m_distanceLeft = getDistanceMetersLeft( );
    m_distanceRight = getDistanceMetersRight( );
    m_wheelSpeeds = getWheelSpeedsMPS( );
    m_odometry.update(Rotation2d.fromDegrees(getGyroHeading( )), m_distanceLeft, m_distanceRight);

    getYawPitchRoll( );

    if (m_driveDebug > 0)
    {
      if (m_validL1)
        m_currentl1 = m_driveL1.getStatorCurrent( );
      if (m_validL2)
        m_currentL2 = m_driveL2.getStatorCurrent( );
      if (m_validR3)
        m_currentR3 = m_driveR3.getStatorCurrent( );
      if (m_validR4)
        m_currentR4 = m_driveR4.getStatorCurrent( );
    }
  }

  private void updateDashboardValues( )
  {
    SmartDashboard.putNumber("DT_distanceLeft", m_distanceLeft);
    SmartDashboard.putNumber("DT_distanceRight", m_distanceRight);
    SmartDashboard.putNumber("DT_wheelSpeedLeft", m_wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("DT_wheelSpeedRight", m_wheelSpeeds.rightMetersPerSecond);
    SmartDashboard.putNumber("DT_currentX", getPose( ).getX( ));
    SmartDashboard.putNumber("DT_currentY", getPose( ).getY( ));
    SmartDashboard.putNumber("DT_heading", getPose( ).getRotation( ).getDegrees( ));

    SmartDashboard.putNumber("DT_gyroYaw", m_yaw);
    SmartDashboard.putNumber("DT_gyroPitch", m_pitch);
    SmartDashboard.putNumber("DT_gyroRoll", m_roll);

    SmartDashboard.putNumber("DT_currentL1", m_currentl1);
    SmartDashboard.putNumber("DT_currentL2", m_currentL2);
    SmartDashboard.putNumber("DT_currentR3", m_currentR3);
    SmartDashboard.putNumber("DT_currentR4", m_currentR4);

    // Only update indicators every 100 ms to cut down on network traffic
    if ((m_periodicInterval++ % 5 == 0) && (m_driveDebug > 1))
      DataLogManager.log(getSubsystem( )
      // @formatter:off
          + ": deg " + getPose( ).getRotation( ).getDegrees( )
          + "LR dist" + m_distanceLeft 
          + " "       + m_distanceRight 
          + " amps (" + String.format("%.1f", m_currentl1) 
          + " "       + String.format("%.1f", m_currentL2) 
          + " "       + String.format("%.1f", m_currentR3) 
          + " "       + String.format("%.1f", m_currentR4) 
          + ")"
          // @formatter:on
      );
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Getters/Setters
  //
  // Wheel encoders
  //
  public void resetEncoders( )
  {
    if (m_validL1)
      m_driveL1.setSelectedSensorPosition(0);
    if (m_validR3)
      m_driveR3.setSelectedSensorPosition(0);
  }

  // Helper methods to convert between meters and native units
  private int metersToNativeUnits(double meters)
  {
    return (int) (meters / DTConsts.kEncoderMetersPerCount);
  }

  private double nativeUnitsToMeters(double nativeUnits)
  {
    return nativeUnits * DTConsts.kEncoderMetersPerCount;
  }

  private int mpsToNativeUnits(double velocity)
  {
    return (int) (velocity / DTConsts.kEncoderMetersPerCount / 10.0);
  }

  private double nativeUnitsToMPS(double nativeUnitsVelocity)
  {
    return nativeUnitsVelocity * DTConsts.kEncoderMetersPerCount * 10;
  }

  private double getDistanceMetersLeft( )
  {
    if (m_validL1)
      return nativeUnitsToMeters(m_driveL1.getSelectedSensorPosition(PIDINDEX));

    return 0;
  }

  private double getDistanceMetersRight( )
  {
    if (m_validR3)
      return nativeUnitsToMeters(m_driveR3.getSelectedSensorPosition(PIDINDEX));

    return 0;
  }

  private DifferentialDriveWheelSpeeds getWheelSpeedsMPS( )
  {
    double leftVelocity = 0;
    double rightVelocity = 0;

    if (m_validL1)
      leftVelocity = nativeUnitsToMPS(m_driveL1.getSelectedSensorVelocity( ));

    if (m_validR3)
      rightVelocity = nativeUnitsToMPS(m_driveR3.getSelectedSensorVelocity( ));

    return new DifferentialDriveWheelSpeeds(leftVelocity, rightVelocity);
  }

  private double joystickOutputToNative(double output)
  {
    double outputScaling = 1.0;
    return (output * outputScaling * Falcon500.kMaxRPM * Falcon500.kEncoderCPR) / (60.0 * 10.0);
  }

  private void velocityArcadeDrive(double yOutput, double xOutput)
  {
    // define joystickOutputToNative
    double leftOutput = joystickOutputToNative(MathUtil.clamp(yOutput + xOutput, -1.0, 1.0));
    double rightOutput = joystickOutputToNative(MathUtil.clamp(yOutput - xOutput, -1.0, 1.0));

    m_driveL1.set(ControlMode.Velocity, leftOutput);
    m_driveR3.set(ControlMode.Velocity, rightOutput);

    m_diffDrive.feedWatchdog( );
  }

  //
  // Gyro
  //
  public void resetGyro( )
  {
    if (m_pigeonValid)
      m_gyroOffset = -m_gyro.getFusedHeading( );
  }

  private double getGyroHeading( )
  {
    return (m_pigeonValid) ? (m_gyroOffset + m_gyro.getFusedHeading( )) : 0;
  }

  private void getYawPitchRoll( )
  {
    m_yaw = m_gyro.getYaw( );
    m_pitch = m_gyro.getPitch( );
    m_roll = m_gyro.getRoll( );
  }

  //
  // Odometry
  //
  private void resetOdometry(Pose2d pose)
  {
    resetEncoders( );
    resetGyro( );
    m_driveSim.setPose(pose);
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(0.0));
  }

  private Pose2d getPose( )
  {
    return m_odometry.getPoseMeters( );
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Set Talon brake/coast mode
  //
  private void setBrakeMode(boolean brakeMode)
  {
    DataLogManager.log(getSubsystem( ) + ": Mode " + ((brakeMode) ? "BRAKE" : "COAST"));
    SmartDashboard.putBoolean("DT_brakeMode", brakeMode);

    if (m_validL1)
      m_driveL1.setNeutralMode((brakeMode) ? NeutralMode.Brake : NeutralMode.Coast);
    if (m_validL2)
      m_driveL2.setNeutralMode((brakeMode) ? NeutralMode.Brake : NeutralMode.Coast);
    if (m_validR3)
      m_driveR3.setNeutralMode((brakeMode) ? NeutralMode.Brake : NeutralMode.Coast);
    if (m_validR4)
      m_driveR4.setNeutralMode((brakeMode) ? NeutralMode.Brake : NeutralMode.Coast);
  }

  //
  // Voltage-based tank drive
  //
  public void tankDriveVolts(double left, double right)
  {
    if (m_validL1)
      m_driveL1.setVoltage(left);
    if (m_validR3)
      m_driveR3.setVoltage(right);

    m_diffDrive.feedWatchdog( );
  }

  private void syncFollowerPIDFromDashboard( )
  {
    m_ramsetePidKf = SmartDashboard.getNumber("DTR_ramsetePidKf", m_ramsetePidKf);
    m_ramsetePidKp = SmartDashboard.getNumber("DTR_ramsetePidKp", m_ramsetePidKp);
    m_ramsetePidKi = SmartDashboard.getNumber("DTR_ramsetePidKi", m_ramsetePidKi);
    m_ramsetePidKd = SmartDashboard.getNumber("DTR_ramsetePidKd", m_ramsetePidKd);

    if (m_validL1)
    {
      m_driveL1.config_kF(SLOTINDEX, m_ramsetePidKf);
      m_driveL1.config_kP(SLOTINDEX, m_ramsetePidKp);
      m_driveL1.config_kI(SLOTINDEX, m_ramsetePidKi);
      m_driveL1.config_kD(SLOTINDEX, m_ramsetePidKd);
      m_driveL1.selectProfileSlot(SLOTINDEX, PIDINDEX);
    }

    if (m_validR3)
    {
      m_driveR3.config_kF(SLOTINDEX, m_ramsetePidKf);
      m_driveR3.config_kP(SLOTINDEX, m_ramsetePidKp);
      m_driveR3.config_kI(SLOTINDEX, m_ramsetePidKi);
      m_driveR3.config_kD(SLOTINDEX, m_ramsetePidKd);
      m_driveR3.selectProfileSlot(SLOTINDEX, PIDINDEX);
    }
  }

  //
  // Check if speed is below tolerance settings
  //
  public boolean driveIsStopped( )
  {
    boolean leftStopped = m_wheelSpeeds.leftMetersPerSecond <= m_stopTolerance;
    boolean rightStopped = m_wheelSpeeds.rightMetersPerSecond <= m_stopTolerance;

    return (leftStopped && rightStopped);
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Trajectory management
  //
  private void plotTrajectory(Trajectory trajectory)
  {
    m_field.getObject("trajectory").setTrajectory(trajectory);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////// Public Interfaces ///////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////
  //
  // Set quick turn for curvature drive
  //
  public void driveSetQuickTurn(boolean quickTurn)
  {
    m_isQuickTurn = quickTurn;
  }

  public void setDriveSlowMode(boolean driveSlowMode)
  {
    m_driveSlowMode = driveSlowMode;
  }

  public void driveStopMotors( )
  {
    if (m_validL1 || m_validR3)
      m_diffDrive.tankDrive(0.0, 0.0, false);
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Teleop driving mode
  //
  public void driveWithJoysticksInit( )
  {
    setBrakeMode(true);
    m_driveL1.configOpenloopRamp(m_openLoopRamp);
    m_driveR3.configOpenloopRamp(m_openLoopRamp);
  }

  public void driveWithJoysticksExecute(XboxController driverPad)
  {
    double xValue;

    xValue = (Robot.isReal( )) ? driverPad.getRightX( ) : driverPad.getLeftTriggerAxis( );
    double yValue = driverPad.getLeftY( );
    double xOutput = 0.0;
    double yOutput = 0.0;

    // If joysticks report a very small value, then stick has been centered
    if (Math.abs(yValue) < 0.05 && Math.abs(xValue) < 0.05)
      m_throttleZeroed = true;

    // If throttle and steering not centered, use zero outputs until they do
    if (m_throttleZeroed)
    {
      if (m_isQuickTurn)
      {
        xOutput = m_driveQTScaling * (xValue * Math.abs(xValue));
        yOutput = m_driveQTScaling * (yValue * Math.abs(yValue));
      }
      else if (m_driveSlowMode)
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

    if (m_validL1 || m_validR3)
      m_diffDrive.curvatureDrive(yOutput, xOutput, m_isQuickTurn);
  }

  public void driveWithJoysticksEnd( )
  {
    setBrakeMode(false);
    m_driveL1.configOpenloopRamp(0.0);
    m_driveR3.configOpenloopRamp(0.0);
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Limelight driving mode
  //
  public void driveWithLimelightInit(boolean m_endAtTarget)
  {
    // get pid values from dashboard

    m_turnConstant = SmartDashboard.getNumber("DTL_turnConstant", m_turnConstant);
    m_turnPidKp = SmartDashboard.getNumber("DTL_turnPidKp", m_turnPidKp);
    m_turnPidKi = SmartDashboard.getNumber("DTL_turnPidKi", m_turnPidKi);
    m_turnPidKd = SmartDashboard.getNumber("DTL_turnPidKd", m_turnPidKd);
    m_turnMax = SmartDashboard.getNumber("DTL_turnMax", m_turnMax);

    m_throttlePidKp = SmartDashboard.getNumber("DTL_throttlePidKp", m_throttlePidKp);
    m_throttlePidKi = SmartDashboard.getNumber("DTL_throttlePidKi", m_throttlePidKi);
    m_throttlePidKd = SmartDashboard.getNumber("DTL_throttlePidKd", m_throttlePidKd);
    m_throttleMax = SmartDashboard.getNumber("DTL_throttleMax", m_throttleMax);
    m_throttleShape = SmartDashboard.getNumber("DTL_throttleShape", m_throttleShape);

    m_targetAngle = SmartDashboard.getNumber("DTL_targetAngle", m_targetAngle);
    m_setPointDistance = SmartDashboard.getNumber("DTL_setPointDistance", m_setPointDistance);
    m_angleThreshold = SmartDashboard.getNumber("DTL_angleThreshold", m_angleThreshold);
    m_distThreshold = SmartDashboard.getNumber("DTL_distThreshold", m_distThreshold);

    // load in Pid constants to controller
    m_turnPid = new PIDController(m_turnPidKp, m_turnPidKi, m_turnPidKd);
    m_throttlePid = new PIDController(m_throttlePidKp, m_throttlePidKi, m_throttlePidKd);

    RobotContainer rc = RobotContainer.getInstance( );
    rc.m_vision.m_yfilter.reset( );
    rc.m_vision.syncStateFromDashboard( );
  }

  public void driveWithLimelightExecute( )
  {
    RobotContainer rc = RobotContainer.getInstance( );
    boolean tv = rc.m_vision.getTargetValid( );
    double tx = rc.m_vision.getHorizOffsetDeg( );
    double ty = rc.m_vision.getVertOffsetDeg( );

    if (!tv)
    {
      velocityArcadeDrive(0, 0);
      if (m_limelightDebug >= 1)
        DataLogManager.log(getSubsystem( ) + ": DTL TV-FALSE - SIT STILL");
      return;
    }

    // get turn value - just horizontal offset from target
    double turnOutput = -m_turnPid.calculate(tx, m_targetAngle);

    if (turnOutput > 0)
      turnOutput = turnOutput + m_turnConstant;
    else if (turnOutput < 0)
      turnOutput = turnOutput - m_turnConstant;

    // get throttle value
    m_limelightDistance = rc.m_vision.getDistLimelight( );

    double throttleDistance = m_throttlePid.calculate(m_limelightDistance, m_setPointDistance);
    double throttleOutput = throttleDistance * Math.pow(Math.cos(turnOutput * Math.PI / 180), m_throttleShape);

    // put turn and throttle outputs on the dashboard
    SmartDashboard.putNumber("DTL_turnOutput", turnOutput);
    SmartDashboard.putNumber("DTL_throttleOutput", throttleOutput);
    SmartDashboard.putNumber("DTL_limeLightDist", m_limelightDistance);

    // cap max turn and throttle output
    turnOutput = MathUtil.clamp(turnOutput, -m_turnMax, m_turnMax);
    throttleOutput = MathUtil.clamp(throttleOutput, -m_throttleMax, m_throttleMax);

    // put turn and throttle outputs on the dashboard
    SmartDashboard.putNumber("DTL_turnClamped", turnOutput);
    SmartDashboard.putNumber("DTL_throttleClamped", throttleOutput);

    if (m_validL1 || m_validR3)
      velocityArcadeDrive(throttleOutput, turnOutput);

    if (m_limelightDebug >= 1)
      DataLogManager.log(getSubsystem( )
      // @formatter:off
          + ": DTL tv: " + tv 
          + " tx: "      + String.format("%.1f", tx)
          + " ty: "      + String.format("%.1f", ty)
          + " lldist: "  + String.format("%.1f", m_limelightDistance)
          + " distErr: " + String.format("%.1f", Math.abs(m_setPointDistance - m_limelightDistance))
          + " stopped: " + driveIsStopped( )
          + " trnOut: "  + String.format("%.2f", turnOutput)
          + " thrOut: "  + String.format("%.2f", throttleOutput)
      // @formatter:on
      );
  }

  public boolean driveWithLimelightIsFinished( )
  {
    RobotContainer rc = RobotContainer.getInstance( );
    boolean tv = rc.m_vision.getTargetValid( );
    double tx = rc.m_vision.getHorizOffsetDeg( );

    if (tv)
    {
      if (Math.abs(tx) <= m_angleThreshold)
        rc.m_led.setLLColor(LEDColor.LEDCOLOR_GREEN);
      else
      {
        if (tx < -m_angleThreshold)
          rc.m_led.setLLColor(LEDColor.LEDCOLOR_RED);
        else if (tx > m_angleThreshold)
          rc.m_led.setLLColor(LEDColor.LEDCOLOR_BLUE);
      }
    }
    else
      rc.m_led.setLLColor(LEDColor.LEDCOLOR_YELLOW);

    return (tv //
        && ((Math.abs(tx)) <= m_angleThreshold) //
        && (Math.abs(m_setPointDistance - m_limelightDistance) <= m_distThreshold)//
        && driveIsStopped( ));
  }

  public void driveWithLimelightEnd( )
  {
    if (m_validL1 || m_validR3)
      velocityArcadeDrive(0.0, 0.0);

    RobotContainer.getInstance( ).m_led.setLLColor(LEDColor.LEDCOLOR_OFF);
  }

  // TODO (remove this later): Previously called LimelightSanityCheck

  public boolean isLimelightValid(double horizAngleRange, double distRange)
  {
    // check whether target is valid
    // check whether the limelight tx and ty is within a certain tolerance
    // check whether distance is within a certain tolerance
    RobotContainer rc = RobotContainer.getInstance( );
    boolean tv = rc.m_vision.getTargetValid( );
    double tx = rc.m_vision.getHorizOffsetDeg( );
    double ty = rc.m_vision.getVertOffsetDeg( );
    m_limelightDistance = rc.m_vision.getDistLimelight( );

    boolean sanityCheck =
        tv && (Math.abs(tx) <= horizAngleRange) && (Math.abs(m_setPointDistance - m_limelightDistance) <= distRange);
    // && (fabs(ty) <= vertAngleRange)

    DataLogManager.log(getSubsystem( )              //
        + ": DTL tv: " + tv                         //
        + " tx: " + tx                              //
        + " ty: " + ty                              //
        + " lldist: " + m_limelightDistance         //
        + " distErr: " + Math.abs(m_setPointDistance - m_limelightDistance) //
        + " sanityCheck: " + ((sanityCheck) ? "PASSED" : "FAILED")          //
    );

    return sanityCheck;
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Autonomous mode - Ramsete path follower
  //
  public void driveWithPathFollowerInit(Trajectory trajectory, boolean resetOdometry)
  {
    m_stopTolerance = SmartDashboard.getNumber("DT_stopTolerance", m_stopTolerance);
    m_ramseteB = SmartDashboard.getNumber("DTR_ramseteB", m_ramseteB);
    m_ramseteZeta = SmartDashboard.getNumber("DTR_ramseteZeta", m_ramseteZeta);

    m_ramseteController = new RamseteController(m_ramseteB, m_ramseteZeta);
    m_trajectory = trajectory;

    if (!RobotBase.isReal( ))
      plotTrajectory(m_trajectory);

    List<Trajectory.State> trajStates = new ArrayList<Trajectory.State>( );
    trajStates = m_trajectory.getStates( );
    DataLogManager.log(getSubsystem( ) + ": DTR states: " + trajStates.size( ) + " dur: " + m_trajectory.getTotalTimeSeconds( ));

    if (m_ramseteDebug >= 1)
      syncFollowerPIDFromDashboard( );

    if (m_ramseteDebug >= 2)
      for (int i = 0; i < trajStates.size( ); i++)
      {
        Trajectory.State curState = trajStates.get(i);
        DataLogManager.log(getSubsystem( )
            // formmater:off
            + ": DTR state time: " + String.format("%.3f", curState.timeSeconds)                      //
            + " Vel: " + String.format("%.2f", curState.velocityMetersPerSecond)                      //
            + " Accel: " + String.format("%.2f", curState.accelerationMetersPerSecondSq)              //
            + " Rotation: " + String.format("%.1f", curState.poseMeters.getRotation( ).getDegrees( )) //
        // formatter:on
        );
      }

    m_trajTimer.reset( );
    m_trajTimer.start( );

    // This initializes the odometry (where we are)
    if (resetOdometry)
      resetOdometry(m_trajectory.getInitialPose( ));

    m_field.setRobotPose(getPose( ));
  }

  public void driveWithPathFollowerExecute( )
  {
    // Need to step through the states through the trajectory

    Trajectory.State trajState = m_trajectory.sample(m_trajTimer.get( ));
    Pose2d currentPose = getPose( );

    ChassisSpeeds targetChassisSpeeds = m_ramseteController.calculate(currentPose, trajState);
    DifferentialDriveWheelSpeeds targetWheelSpeed = m_kinematics.toWheelSpeeds(targetChassisSpeeds);

    double targetVelLeft = mpsToNativeUnits(targetWheelSpeed.leftMetersPerSecond);
    double targetVelRight = mpsToNativeUnits(targetWheelSpeed.rightMetersPerSecond);
    double currentVelLeft = mpsToNativeUnits(m_wheelSpeeds.leftMetersPerSecond);
    double currentVelRight = mpsToNativeUnits(m_wheelSpeeds.rightMetersPerSecond);

    double targetTrajX = trajState.poseMeters.getX( );
    double targetTrajY = trajState.poseMeters.getY( );
    double currentTrajX = currentPose.getX( );
    double currentTrajY = currentPose.getY( );

    double targetHeading = trajState.poseMeters.getRotation( ).getDegrees( );
    double currentHeading = currentPose.getRotation( ).getDegrees( );

    if (m_validL1)
      m_driveL1.set(TalonFXControlMode.Velocity, targetVelLeft);
    if (m_validR3)
      m_driveR3.set(TalonFXControlMode.Velocity, targetVelRight);

    m_diffDrive.feedWatchdog( );

    if (m_ramseteDebug >= 1)
      DataLogManager.log(getSubsystem( )
      // @formatter:off
          + ": DTR time: "     + String.format("%.3f", m_trajTimer.get( ))
              + " curXYR: "    + String.format("%.2f", currentTrajX) 
                + " "          + String.format("%.2f", currentTrajY) 
                + " "          + String.format("%.1f", currentHeading)
              + " targXYR: "   + String.format("%.2f", targetTrajX) 
                + " "          + String.format("%.2f", targetTrajY) 
                + " "          + String.format("%.1f", targetHeading)
              + " chasXYO: "   + String.format("%.1f", targetChassisSpeeds.vxMetersPerSecond) 
                + " "          + String.format("%.1f", targetChassisSpeeds.vyMetersPerSecond)
                + " "          + String.format("%.1f", targetChassisSpeeds.omegaRadiansPerSecond)
              + " targVelLR: " + String.format("%.1f", targetWheelSpeed.leftMetersPerSecond) 
                + " "          + String.format("%.1f", targetWheelSpeed.rightMetersPerSecond )
              + " curVelLR: "  + String.format("%.2f", nativeUnitsToMPS(currentVelLeft)) 
                + " "          + String.format("%.2f", nativeUnitsToMPS(currentVelRight)) 
        // @formatter:on
      );

    if (m_ramseteDebug >= 2)
    {
      // target velocity and its error
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_targetVelLeft", targetVelLeft);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_targetVelRight", targetVelRight);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_currentVelLeft", currentVelLeft);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_currentVelRight", currentVelRight);

      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_velErrorLeft", targetVelLeft - currentVelLeft);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_velErrorRight", targetVelRight - currentVelRight);

      // target distance and its error
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_currentTrajX", targetTrajX);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_currentTrajY", targetTrajY);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_targetTrajX", currentTrajX);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_targetTrajY", currentTrajY);

      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_trajErrorX", trajState.poseMeters.relativeTo(currentPose).getX( ));
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_trajErrorY", trajState.poseMeters.relativeTo(currentPose).getY( ));

      // target heading and its error
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_targetHeading", targetHeading);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_currentHeading", currentHeading);
      SmartDashboard.putNumber(getSubsystem( ) + ": DTR_headingError",
          trajState.poseMeters.relativeTo(currentPose).getRotation( ).getDegrees( ));
    }
  }

  public boolean driveWithPathFollowerIsFinished( )
  {
    if (m_trajTimer.get( ) == 0)
      return false;

    if (m_trajTimer.get( ) >= 15.0)
    {
      DataLogManager.log(getName( ) + ": path follower timeout!");
      return true;
    }

    return ((m_trajTimer.get( ) >= m_trajectory.getTotalTimeSeconds( ))
        && (Math.abs(m_wheelSpeeds.leftMetersPerSecond) <= 0 + m_stopTolerance)
        && (Math.abs(m_wheelSpeeds.rightMetersPerSecond) <= 0 + m_stopTolerance));
  }

  public void driveWithPathFollowerEnd( )
  {
    m_trajTimer.stop( );
    velocityArcadeDrive(0.0, 0.0);
  }
}
