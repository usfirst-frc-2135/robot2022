
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.CLConsts;
import frc.robot.Constants.CLConsts.Height;
import frc.robot.Constants.CLConsts.Position;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.frc2135.PhoenixUtil;
import frc.robot.frc2135.RobotConfig;

/**
 *
 */
public class Climber extends SubsystemBase
{
  // Constants

  private Solenoid                        m_gatehook            = new Solenoid(0, PneumaticsModuleType.CTREPCM, 1);
  private WPI_TalonFX                     m_motorCL14           = new WPI_TalonFX(14);
  private WPI_TalonFX                     m_motorCL15           = new WPI_TalonFX(15);
  private WPI_CANCoder                    m_gatehookAngle       = new WPI_CANCoder(0);
  private DigitalInput                    m_climberDownLeft     = new DigitalInput(0);
  private DigitalInput                    m_climberDownRight    = new DigitalInput(1);

  // Declare constants
  private final int                       m_climberDebug        = 0; // DEBUG flag to disable/enable extra logging calls
  private final int                       kCANTimeout           = 30;   // CAN timeout in msec to wait for response

  // Declare module variables

  private boolean                         m_talonValidCL14; // Health indicator for climber Talon 14
  private boolean                         m_talonValidCL15; // Health indicator for climber Talon 15

  private double                          m_stickDeadband       = 0.2;
  private int                             m_resetCountCL14; // reset counter for motor
  private int                             m_resetCountCL15; // reset counter for motor

  private double                          m_targetInches;   // Target inches of height that are requested of the climber
  private double                          m_curInches;      // Current elevator height in inches
  private boolean                         m_calibrated;       // Indicates whether the climber has been calibrated
  private boolean                         m_isMoving            = false; // State of whether the climber is moving or
                                                                         // stationary

  private Timer                           m_safetyTimer; // Safety timer for use in elevator
  private double                          m_safetyTimeout; // Seconds that the timer ran before stopping

  // Config file parameters
  private int                             m_velocity;           // Climber motion velocity
  private int                             m_acceleration;       // Climber motion acceleration
  private int                             m_sCurveStrength;     // Climber motion S curve smoothing strength
  private double                          m_pidKf;           // Climber PID force constant
  private double                          m_pidKp;           // Climber PID proportional constant
  private double                          m_pidKi;           // Climber PID integral constant
  private double                          m_pidKd;           // Climber PID derivative constant
  private int                             m_CLAllowedError;     // Climber PID allowable closed loop error in counts
  private double                          m_toleranceInches; // Climber PID tolerance in inches

  private double                          m_climberMaxHeight; // Climber maximum allowable height
  private double                          m_climberMinHeight; // Climber minimum allowable height

  private double                          m_stowHeight;         // 0.25 inches
  private double                          m_extendL2;           // 29 inches
  private double                          m_rotateL3;           // 21 inches
  private double                          m_extendL3;           // 31.5 inches
  private double                          m_gatehookRestHeight; // 0.35 inches
  private double                          m_raiseL4;            // 25.25 inches
  private double                          m_currentLeftHeight;  // Current height encoder read from hall sensors
  private double                          m_currentRightHeight;

  private double                          m_yaw;
  private double                          m_pitch;
  private double                          m_roll;

  // Current Limit Settings
  private SupplyCurrentLimitConfiguration m_supplyCurrentLimits = new SupplyCurrentLimitConfiguration(true, 45.0, 45.0, 0.001);
  private StatorCurrentLimitConfiguration m_statorCurrentLimits = new StatorCurrentLimitConfiguration(true, 80.0, 80.0, 0.001);

  // Periodic declarations
  public int                              periodicInterval      = 0;
  public Position                         state                 = Position.CLIMBER_INIT;

  /**
   *
   */
  public Climber( )
  {
    // Set the names for this subsystem for later use
    setName("Climber");
    setSubsystem("Climber");
    addChild("GateHook", m_gatehook);

    addChild("Brake", m_gatehook);

    // Validate Talon SRX controllers, initialize and display firmware versions
    m_talonValidCL14 = PhoenixUtil.getInstance( ).talonFXInitialize(m_motorCL14, "CL14");
    m_talonValidCL15 = PhoenixUtil.getInstance( ).talonFXInitialize(m_motorCL15, "CL15");

    SmartDashboard.putBoolean("CL_CL14Valid", m_talonValidCL14);
    SmartDashboard.putBoolean("CL_CL15Valid", m_talonValidCL15);

    DataLogManager.log(getSubsystem( ) + "CL 14 motor valid: {}" + m_talonValidCL14);
    DataLogManager.log(getSubsystem( ) + "CL 15 motor valid: {}" + m_talonValidCL15);

    // Check if solenoids are functional or blacklisted
    if (m_gatehook.isDisabled( ))
      DataLogManager.log("CL Climber Solenoid is BLACKLISTED");
    else
      DataLogManager.log("CL Climber Solenoid is FUNCTIONAL");

    // Initialize Variables
    RobotConfig config = RobotConfig.getInstance( );
    m_velocity = config.getValueAsInt("CL_Velocity", 21776);
    m_acceleration = config.getValueAsInt("CL_Acceleration", 43552);
    m_sCurveStrength = config.getValueAsInt("CL_SCurveStrength", 0);
    m_pidKf = config.getValueAsDouble("CL_PidKf", 0.0);
    m_pidKp = config.getValueAsDouble("CL_PidKp", 0.0);
    m_pidKi = config.getValueAsDouble("CL_PidKi", 0.000);
    m_pidKd = config.getValueAsDouble("CL_PidKd", 0.000);
    m_CLAllowedError = config.getValueAsInt("CL_CLAllowedError", 0);
    m_toleranceInches = config.getValueAsDouble("CL_ToleranceInches", 0.25);
    m_climberMaxHeight = config.getValueAsDouble("CL_MaxHeight", 36.0);
    m_climberMinHeight = config.getValueAsDouble("CL_MinHeight", 0.0);
    m_stowHeight = config.getValueAsDouble("CL_StowHeight", 0.25);
    m_extendL2 = config.getValueAsDouble("CL_ExtendL2", 29.0);
    m_rotateL3 = config.getValueAsDouble("CL_RotateL3", 31.25);
    m_gatehookRestHeight = config.getValueAsDouble("CL_GatehookRestHeight", 0.35);
    m_raiseL4 = config.getValueAsDouble("CL_RaiseL4", 25.25);

    SmartDashboard.putNumber("CL_Velocity", m_velocity);
    SmartDashboard.putNumber("CL_Acceleration", m_acceleration);
    SmartDashboard.putNumber("CL_SCurveStrength", m_sCurveStrength);
    SmartDashboard.putNumber("CL_PidKf", m_pidKf);
    SmartDashboard.putNumber("CL_PidKp", m_pidKp);
    SmartDashboard.putNumber("CL_PidKi", m_pidKi);
    SmartDashboard.putNumber("CL_PidKd", m_pidKd);
    SmartDashboard.putNumber("CL_StowHeight", m_stowHeight);
    SmartDashboard.putNumber("CL_ExtendL2", m_extendL2);
    SmartDashboard.putNumber("CL_RotateL3", m_rotateL3);
    SmartDashboard.putNumber("CL_GatehookRestHeight", m_gatehookRestHeight);
    SmartDashboard.putNumber("CL_RaiseL4", m_raiseL4);

    // Magic Motion variables
    m_curInches = 0.0;
    m_targetInches = 0.0;
    m_calibrated = false;

    // Field for manually progamming climber height
    SmartDashboard.putBoolean("CL_Calibrated", m_calibrated);

    // Set motor directions
    // Turn on Coast mode (not brake)
    // Set motor peak outputs
    if (m_talonValidCL14)
    {
      m_motorCL14.setInverted(true);
      m_motorCL14.setNeutralMode(NeutralMode.Brake);
      m_motorCL14.setSafetyEnabled(false);

      m_motorCL14.configVoltageCompSaturation(12.0, 0);
      m_motorCL14.enableVoltageCompensation(true);

      PhoenixUtil.getInstance( ).checkError(m_motorCL14.configSupplyCurrentLimit(m_supplyCurrentLimits),
          "CL14 ConfigSupplyCurrentLimits");
      PhoenixUtil.getInstance( ).checkError(m_motorCL14.configStatorCurrentLimit(m_statorCurrentLimits),
          "CL14 ConfigStatorCurrentLimits");
      // Configure Magic Motion settings
      // TODO: replace selectProfileSlot()
      // PhoenixUtil.getInstance().checkError(m_motorCL14.selectProfileSlot(0, 0), "CL14
      // SelectProfileSlot");

      // Configure sensor settings
      PhoenixUtil.getInstance( ).checkError(m_motorCL14.setSelectedSensorPosition(0, 0, kCANTimeout),
          "CL14 SetSelectedSensorPosition");

      // Set allowable closed loop error
      PhoenixUtil.getInstance( ).checkError(m_motorCL14.configAllowableClosedloopError(0, m_CLAllowedError, kCANTimeout),
          "CL14 ConfigAllowableClosedloopError");

      PhoenixUtil.getInstance( ).checkError(m_motorCL14.configMotionCruiseVelocity(m_velocity, kCANTimeout),
          "CL14 ConfigMotionCruiseVelocity");
      PhoenixUtil.getInstance( ).checkError(m_motorCL14.configMotionAcceleration(m_acceleration, kCANTimeout),
          "CL14 ConfigMotionAcceleration");
      PhoenixUtil.getInstance( ).checkError(m_motorCL14.configMotionSCurveStrength(m_sCurveStrength, kCANTimeout),
          "CL14 ConfigMotionSCurveStrength");
      PhoenixUtil.getInstance( ).checkError(m_motorCL14.config_kF(0, m_pidKf, kCANTimeout), "CL14 Config_kF");
      PhoenixUtil.getInstance( ).checkError(m_motorCL14.config_kP(0, m_pidKp, kCANTimeout), "CL14 Config_kP");
      PhoenixUtil.getInstance( ).checkError(m_motorCL14.config_kI(0, m_pidKi, kCANTimeout), "CL14 Config_kI");
      PhoenixUtil.getInstance( ).checkError(m_motorCL14.config_kD(0, m_pidKd, kCANTimeout), "CL14 Config_KD");

      m_motorCL14.set(ControlMode.PercentOutput, 0.0);
    }

    // FollowerInitialize();
    if (m_talonValidCL15)
    {
      m_motorCL15.setInverted(false);
      m_motorCL15.setNeutralMode(NeutralMode.Brake);
      m_motorCL15.setSafetyEnabled(false);

      m_motorCL15.configVoltageCompSaturation(12.0, 0);
      m_motorCL15.enableVoltageCompensation(true);

      PhoenixUtil.getInstance( ).checkError(m_motorCL15.configSupplyCurrentLimit(m_supplyCurrentLimits),
          "CL15 ConfigSupplyCurrentLimit");
      PhoenixUtil.getInstance( ).checkError(m_motorCL15.configStatorCurrentLimit(m_statorCurrentLimits),
          "CL15 ConfigStatorCurrentLimit");

      // Configure Magic Motion settings
      // TODO: fix error
      // PhoenixUtil.getInstance().checkError(m_motorCL15.selectProfileSlot(0, 0), "CL15
      // SelectProfileSlot");

      // Configure sensor settings
      PhoenixUtil.getInstance( ).checkError(m_motorCL15.setSelectedSensorPosition(0, 0, kCANTimeout),
          "CL15 SetSelectedSensorPosition");

      // Set allowable closed loop error
      PhoenixUtil.getInstance( ).checkError(m_motorCL15.configAllowableClosedloopError(0, m_CLAllowedError, kCANTimeout),
          "CL15 ConfigAllowableClosedloopError");

      PhoenixUtil.getInstance( ).checkError(m_motorCL15.configMotionCruiseVelocity(m_velocity, kCANTimeout),
          "CL14 ConfigMotionCruiseVelocity");
      PhoenixUtil.getInstance( ).checkError(m_motorCL15.configMotionAcceleration(m_acceleration, kCANTimeout),
          "CL14 ConfigMotionAcceleration");
      PhoenixUtil.getInstance( ).checkError(m_motorCL15.configMotionSCurveStrength(m_sCurveStrength, kCANTimeout),
          "CL14 ConfigMotionSCurveStrength");
      PhoenixUtil.getInstance( ).checkError(m_motorCL15.config_kF(0, m_pidKf, kCANTimeout), "CL15 Config_kF");
      PhoenixUtil.getInstance( ).checkError(m_motorCL15.config_kP(0, m_pidKp, kCANTimeout), "CL15 Config_kP");
      PhoenixUtil.getInstance( ).checkError(m_motorCL15.config_kI(0, m_pidKi, kCANTimeout), "CL15 Config_kI");
      PhoenixUtil.getInstance( ).checkError(m_motorCL15.config_kD(0, m_pidKd, kCANTimeout), "CL15 Config_KD");

      m_motorCL15.set(ControlMode.PercentOutput, 0.0);
    }
    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
    // Put code here to be run every loop

    double outputCL14 = 0.0;
    double outputCL15 = 0.0;

    // if disabled
    if (RobotState.isDisabled( ))
    {
      RobotContainer robotContainer = RobotContainer.getInstance( );
      if (!m_climberDownLeft.get( ) || !m_climberDownRight.get( ))
      {
        robotContainer.m_led.setColor(LEDColor.LEDCOLOR_BLUE);
      }
      else
      {
        robotContainer.m_led.setColor(LEDColor.LEDCOLOR_OFF);
      }
    }

    if (m_talonValidCL14)
      outputCL14 = m_motorCL14.getMotorOutputPercent( );
    SmartDashboard.putNumber("CL_Output_CL14", outputCL14);

    if (m_talonValidCL15)
      outputCL15 = m_motorCL15.getMotorOutputPercent( );
    SmartDashboard.putNumber("CL_Output_CL15", outputCL15);

    int curCounts = 0;
    if (m_talonValidCL14)
      curCounts = (int) m_motorCL14.getSelectedSensorPosition(0);

    // bool CL15FollowerMode = (m_motorCL15.GetControlMode() == ControlMode::Follower);
    // SmartDashboard.putBoolean("CL_CL15FollowerMode", CL15FollowerMode);
    // if (!CL15FollowerMode)
    // spdlog::error("CL15 is not in Follower Mode");

    m_curInches = countsToInches(curCounts);
    SmartDashboard.putNumber("CL_Height", m_curInches);

    // Calibrate climber if hall sensors are activated
    // if (!m_climberDownLeft.Get() || !m_climberDownRight.Get())
    // Calibrate();

    // Only update indicators every 100 ms to cut down on network traffic
    if (periodicInterval++ % 5 == 0)
    {
      if (m_climberDebug > 0)
      {
        double currentCL14 = 0.0;
        double currentCL15 = 0.0;

        if (m_talonValidCL14)
          // TODO: should we replace getOutputCurrent to getStatorCurrent again?
          currentCL14 = m_motorCL14.getOutputCurrent( );

        if (m_talonValidCL15)
          currentCL15 = m_motorCL15.getOutputCurrent( );

        SmartDashboard.putNumber("CL_Current_CL14", currentCL14);
        SmartDashboard.putNumber("CL_Current_CL15", currentCL15);
      }
    }
    if (m_motorCL14.hasResetOccurred( ))
    {
      m_resetCountCL14 += 1;
      SmartDashboard.putNumber("HL_Resets_CL14", m_resetCountCL14);
    }
    if (m_motorCL15.hasResetOccurred( ))
    {
      m_resetCountCL15 += 1;
      SmartDashboard.putNumber("HL_Resets_CL15", m_resetCountCL15);
    }
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run when in simulation
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public void initialize( )
  {
    double curCounts = 0.0;

    DataLogManager.log(getSubsystem( ) + ": subsystem initialized!");

    setGateHook(false);

    setClimberStopped( );

    if (m_talonValidCL14)
      curCounts = m_motorCL14.getSelectedSensorPosition(0);

    m_curInches = countsToInches((int) curCounts);
    m_targetInches = m_curInches;
    m_isMoving = false;
    DataLogManager.log(getSubsystem( ) + ": Init Target Inches: {}" + m_targetInches);
  }

  // Dump all Talon faults
  public void faultDump( )
  {
    PhoenixUtil.getInstance( ).talonFXFaultDump(m_motorCL14, "CL 14");
    PhoenixUtil.getInstance( ).talonFXFaultDump(m_motorCL15, "CL 15");
  }

  public void moveClimberWithJoysticks(XboxController joystick)
  {

    double yCLValue = 0.0;
    double motorOutput = 0.0;

    yCLValue = -joystick.getLeftY( );
    if (yCLValue > -0.1 && yCLValue < 0.1)
    {
      if (state != Constants.CLConsts.Position.CLIMBER_STOPPED)
        DataLogManager.log(getSubsystem( ) + "CL Climber Stopped");
      state = Constants.CLConsts.Position.CLIMBER_STOPPED;
    }
    else
    {
      // If joystick is above a value, climber will move up
      if (yCLValue > m_stickDeadband)
      {
        if (state != Constants.CLConsts.Position.CLIMBER_UP)
          DataLogManager.log(getSubsystem( ) + ("CL Climber Up"));
        state = Constants.CLConsts.Position.CLIMBER_UP;

        yCLValue -= m_stickDeadband;
        yCLValue *= (1.0 / (1.0 - m_stickDeadband));
        motorOutput = (0.3) * (yCLValue * Math.abs(yCLValue));
      }
      // If joystick is below a value, climber will move down
      else if (yCLValue < -m_stickDeadband)
      {
        if (state != Constants.CLConsts.Position.CLIMBER_DOWN)
          DataLogManager.log(getSubsystem( ) + "CL Climber Down");
        state = Constants.CLConsts.Position.CLIMBER_DOWN;

        yCLValue += m_stickDeadband;
        yCLValue *= (1.0 / (1.0 - m_stickDeadband));
        motorOutput = (0.3) * (yCLValue * Math.abs(yCLValue));
      }
    }

    if (m_talonValidCL14)
      m_motorCL14.set(ControlMode.PercentOutput, motorOutput);

    if (m_talonValidCL15)
      m_motorCL15.set(ControlMode.PercentOutput, motorOutput);
  }

  public void setClimberStopped( )
  {
    DataLogManager.log(getSubsystem( ) + ": CL Set Climber Stopped");

    if (m_talonValidCL14)
      m_motorCL14.set(ControlMode.PercentOutput, 0);

    if (m_talonValidCL15)
      m_motorCL15.set(ControlMode.PercentOutput, 0);
  }

  public void setGateHook(boolean hookClosed)
  {
    if (hookClosed != m_gatehook.get( ))
    {
      DataLogManager.log(getSubsystem( ) + ": CL HOOK {}" + ((hookClosed) ? "OPEN" : "CLOSED"));
      SmartDashboard.putBoolean("CL_Hook_Closed", hookClosed);

      m_gatehook.set(hookClosed);
    }
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

  public double inchesToCounts(double inches)
  {
    return inches / Constants.CLConsts.kInchesPerCount;
  }

  public double countsToInches(int counts)
  {
    return counts * Constants.CLConsts.kInchesPerCount;
  }

  public void moveToCalibrate( )
  {
    if (m_talonValidCL14)
      m_motorCL14.set(ControlMode.PercentOutput, -0.1);

    if (m_talonValidCL15)
      m_motorCL15.set(ControlMode.PercentOutput, -0.1);
  }

  void calibrate( )
  {
    if (m_talonValidCL14)
      m_motorCL14.setSelectedSensorPosition(0, 0, kCANTimeout);

    if (m_talonValidCL15)
      m_motorCL15.setSelectedSensorPosition(0, 0, kCANTimeout);

    m_targetInches = 0;
    m_curInches = 0;
    m_calibrated = true;
    SmartDashboard.putBoolean("CL_Calibrated", m_calibrated);
  }

  void moveClimberDistanceInit(Height state)
  {
    if (m_climberDebug != 0)
    {
      m_pidKf = SmartDashboard.getNumber("CL_PidKf", m_pidKf);
      m_velocity = (int) SmartDashboard.getNumber("CL_Velocity", m_velocity);
      m_acceleration = (int) SmartDashboard.getNumber("CL_Acceleration", m_acceleration);
      m_sCurveStrength = (int) SmartDashboard.getNumber("CL_SCurveStrength", m_sCurveStrength);
      m_pidKp = SmartDashboard.getNumber("CL_PidKp", m_pidKp);
      m_pidKi = SmartDashboard.getNumber("CL_PidKi", m_pidKi);
      m_pidKd = SmartDashboard.getNumber("CL_PidKd", m_pidKd);

      m_motorCL14.config_kF(0, m_pidKf, 0);
      m_motorCL14.configMotionCruiseVelocity(m_velocity, 0);
      m_motorCL14.configMotionAcceleration(m_acceleration, 0);
      m_motorCL14.configMotionSCurveStrength(m_sCurveStrength, 0);
      m_motorCL14.config_kP(0, m_pidKp, 0);
      m_motorCL14.config_kI(0, m_pidKi, 0);
      m_motorCL14.config_kD(0, m_pidKd, 0);

      m_motorCL15.config_kF(0, m_pidKf, 0);
      m_motorCL15.configMotionCruiseVelocity(m_velocity, 0);
      m_motorCL15.configMotionAcceleration(m_acceleration, 0);
      m_motorCL15.configMotionSCurveStrength(m_sCurveStrength, 0);
      m_motorCL15.config_kP(0, m_pidKp, 0);
      m_motorCL15.config_kI(0, m_pidKi, 0);
      m_motorCL15.config_kD(0, m_pidKd, 0);
    }

    switch (state)
    {
      case NOCHANGE_HEIGHT : // Do not change from current level!
        m_targetInches = m_curInches;
        if (m_targetInches < 0.25)
          m_targetInches = 0.25;
        break;
      case STOW_HEIGHT :
        m_targetInches = SmartDashboard.getNumber("CL_StowHeight", m_stowHeight);
        break;
      case EXTEND_L2_HEIGHT :
        m_targetInches = SmartDashboard.getNumber("CL_ExtendL2", m_extendL2);
        break;
      case ROTATE_L3_HEIGHT :
        m_targetInches = SmartDashboard.getNumber("CL_RotateL3", m_rotateL3);
        break;
      case GATEHOOK_REST_HEIGHT :
        m_targetInches = SmartDashboard.getNumber("CL_GatehookRestHeight", m_gatehookRestHeight);
        break;
      case RAISE_L4_HEIGHT :
        m_targetInches = SmartDashboard.getNumber("CL_RaiseL4", m_raiseL4);
        break;
      default :
        DataLogManager.log(getSubsystem( ) + ": requested height is invalid - " + state);
        return;
    }
  }
}
