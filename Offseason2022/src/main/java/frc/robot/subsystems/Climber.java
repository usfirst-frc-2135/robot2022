
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CLConsts;
import frc.robot.Constants.CLConsts.CLMode;
import frc.robot.Constants.CLConsts.Height;
import frc.robot.Constants.Falcon500;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.RobotContainer;
import frc.robot.frc2135.PhoenixUtil;

/**
 *
 */
public class Climber extends SubsystemBase
{
  // Constants
  private static final int                CANTIMEOUT            = 30;  // CAN timeout in msec
  private static final int                PIDINDEX              = 0;   // PID in use (0-primary, 1-aux)
  private static final int                SLOTINDEX             = 0;   // Use first PID slot

  // Devices and simulation objects
  private WPI_TalonFX                     m_motorCL14           = new WPI_TalonFX(CLConsts.kCL14LeftCANID);
  private WPI_TalonFX                     m_motorCL15           = new WPI_TalonFX(CLConsts.kCL15RightCANID);
  private DigitalInput                    m_climberDownLeft     = new DigitalInput(CLConsts.kCLLeftLimitDIO);
  private DigitalInput                    m_climberDownRight    = new DigitalInput(CLConsts.kCLRightLimitDIO);
  private Solenoid                        m_gateHook            =
      new Solenoid(0, PneumaticsModuleType.CTREPCM, CLConsts.kGateHookSolenod);
  private CANCoder                        m_gateHookAngle       = new CANCoder(CLConsts.kCLCancoderID);

  private TalonFXSimCollection            m_motorCL14Sim        = new TalonFXSimCollection(m_motorCL14);
  private ElevatorSim                     m_elevatorCL14Sim     =
      new ElevatorSim(DCMotor.getFalcon500(1), CLConsts.kClimberGearRatio, 1.0, 0.0175, 0.0, 1.0, VecBuilder.fill(0.01));

  private SupplyCurrentLimitConfiguration m_supplyCurrentLimits = new SupplyCurrentLimitConfiguration(true,
      Falcon500.kSupplyCurrentLimit, Falcon500.kSupplyTriggerCurrent, Falcon500.kSupplyTriggerTime);
  private StatorCurrentLimitConfiguration m_statorCurrentLimits = new StatorCurrentLimitConfiguration(true,
      Falcon500.kStatorCurrentLimit, Falcon500.kStatorTriggerCurrent, Falcon500.kStatorTriggerTime);

  // Declare module variables
  private int                             m_velocity            = CLConsts.kMMVelocity;        // motion magic velocity
  private int                             m_acceleration        = CLConsts.kMMAcceleration;    // motion magic acceleration
  private int                             m_sCurveStrength      = CLConsts.kMMSCurveStrength;  // motion magic S curve smoothing
  private double                          m_pidKf               = CLConsts.kCLPidKf;           // PID force constant
  private double                          m_pidKp               = CLConsts.kCLPidKp;           // PID proportional
  private double                          m_pidKi               = CLConsts.kCLPidKi;           // PID integral
  private double                          m_pidKd               = CLConsts.kCLPidKd;           // PID derivative
  private int                             m_CLAllowedError      = CLConsts.kCLAllowedError;    // PID allowable closed loop error
  private double                          m_toleranceInches     = CLConsts.kCLToleranceInches; // PID tolerance in inches

  private double                          m_climberMaxHeight    = CLConsts.kClimberMaxHeight;  // maximum allowable height
  private double                          m_climberMinHeight    = CLConsts.kClimberMinHeight;  // minimum allowable height

  private double                          m_stowHeight          = CLConsts.kStowHeight;         // Stow height
  private double                          m_extendL2            = CLConsts.kExtendL2;           // Extend to layout 2
  private double                          m_rotateL3            = CLConsts.kRotateL3;           // Rotate to layout 3
  private double                          m_raiseL4             = CLConsts.kRaiseL4;            // Raise to layout 4
  private double                          m_gatehookRestHeight  = CLConsts.kGatehookRestHeight; // Gate hook resting height

  private int                             m_climberDebug        = 0; // DEBUG flag to disable/enable extra logging calls
  private boolean                         m_validCL14;               // Health indicator for climber Talon 14
  private boolean                         m_validCL15;               // Health indicator for climber Talon 15
  private int                             m_resetCountCL14;          // reset counter for motor
  private int                             m_resetCountCL15;          // reset counter for motor

  private double                          m_stickDeadband       = 0.2;

  private CLMode                          state                 = CLMode.CLIMBER_INIT;
  private boolean                         m_calibrated          = false;  // Indicates whether the climber has been calibrated
  private double                          m_targetInches        = 0.0;    // Target height in inches requested
  private double                          m_curInches           = 0.0;    // Current elevator height in inches

  private Timer                           m_safetyTimer; // Safety timer for use in elevator
  private double                          m_safetyTimeout; // Seconds that the timer ran before stopping

  /**
   *
   */
  public Climber( )
  {
    // Set the names for this subsystem for later use
    setName("Climber");
    setSubsystem("Climber");
    addChild("GateHook", m_gateHook);
    addChild("DownLimitLeft", m_climberDownLeft);
    addChild("DownLimitLeft", m_climberDownRight);

    // Validate Talon SRX controllers, initialize and display firmware versions
    m_validCL14 = PhoenixUtil.getInstance( ).talonFXInitialize(m_motorCL14, "CL14");
    m_validCL15 = PhoenixUtil.getInstance( ).talonFXInitialize(m_motorCL15, "CL15");
    SmartDashboard.putBoolean("HL_validCL14", m_validCL14);
    SmartDashboard.putBoolean("HL_validCL15", m_validCL15);

    // Check if solenoids are functional or blacklisted
    DataLogManager.log(getSubsystem( ) + ": CL Climber Solenoid is " + ((m_gateHook.isDisabled( )) ? "BLACKLISTED" : "OK"));

    // Initialize Variables
    SmartDashboard.putNumber("CL_velocity", m_velocity);
    SmartDashboard.putNumber("CL_acceleration", m_acceleration);
    SmartDashboard.putNumber("CL_sCurveStrength", m_sCurveStrength);
    SmartDashboard.putNumber("CL_pidKf", m_pidKf);
    SmartDashboard.putNumber("CL_pidKp", m_pidKp);
    SmartDashboard.putNumber("CL_pidKi", m_pidKi);
    SmartDashboard.putNumber("CL_pidKd", m_pidKd);

    SmartDashboard.putNumber("CL_stowHeight", m_stowHeight);
    SmartDashboard.putNumber("CL_extendL2", m_extendL2);
    SmartDashboard.putNumber("CL_rotateL3", m_rotateL3);
    SmartDashboard.putNumber("CL_raiseL4", m_raiseL4);
    SmartDashboard.putNumber("CL_gatehookRestHeight", m_gatehookRestHeight);

    // Field for manually progamming climber height
    SmartDashboard.putNumber("CL_curInches", m_curInches);
    SmartDashboard.putNumber("CL_targetInches", m_targetInches);
    SmartDashboard.putBoolean("CL_calibrated", m_calibrated);

    // Set motor directions, coast mode (not brake), set motor peak outputs
    if (m_validCL14)
      climberTalonInitialize(m_motorCL14, true);
    if (m_validCL15)
      climberTalonInitialize(m_motorCL15, false);

    m_gateHookAngle.configMagnetOffset(0.0, CANTIMEOUT);

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    // if disabled, set LED when down
    if (RobotState.isDisabled( ))
    {
      if (!m_climberDownLeft.get( ) || !m_climberDownRight.get( ))
        RobotContainer.getInstance( ).m_led.setNormalColor(LEDColor.LEDCOLOR_BLUE);
      else
        RobotContainer.getInstance( ).m_led.setNormalColor(LEDColor.LEDCOLOR_OFF);
    }

    double outputCL14 = 0.0;
    double outputCL15 = 0.0;
    double currentCL14 = 0.0;
    double currentCL15 = 0.0;

    SmartDashboard.putNumber("CL_gateHookAngle", m_gateHookAngle.getAbsolutePosition( ));

    if (m_validCL14)
    {
      int curCounts = (int) m_motorCL14.getSelectedSensorPosition(0);
      m_curInches = countsToInches(curCounts);
      SmartDashboard.putNumber("CL_curInches", m_curInches);

      m_resetCountCL14 += (m_motorCL14.hasResetOccurred( )) ? 1 : 0;
      SmartDashboard.putNumber("HL_resetCountCL14", m_resetCountCL14);

      outputCL14 = m_motorCL14.getMotorOutputPercent( );
      SmartDashboard.putNumber("CL_outputCL14", outputCL14);

      if (m_climberDebug > 0)
      {
        currentCL14 = m_motorCL14.getStatorCurrent( );
        SmartDashboard.putNumber("CL_currentCL14", currentCL14);
      }
    }

    if (m_validCL15)
    {
      m_resetCountCL15 += (m_motorCL15.hasResetOccurred( )) ? 1 : 0;
      SmartDashboard.putNumber("HL_resetCountCL15", m_resetCountCL15);

      outputCL15 = m_motorCL15.getMotorOutputPercent( );
      SmartDashboard.putNumber("CL_outputCL15", outputCL15);

      if (m_climberDebug > 0)
      {
        currentCL15 = m_motorCL15.getStatorCurrent( );
        SmartDashboard.putNumber("CL_currentCL15", currentCL15);
      }
    }
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run when in simulation

    // Set input flywheel voltage from the motor setting
    m_motorCL14Sim.setBusVoltage(RobotController.getInputVoltage( ));
    m_elevatorCL14Sim.setInput(m_motorCL14Sim.getMotorOutputLeadVoltage( ));

    // update for 20 msec loop
    m_elevatorCL14Sim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_motorCL14Sim.setIntegratedSensorVelocity(elevatorRPMToNative(m_elevatorCL14Sim.getVelocityMetersPerSecond( )));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorCL14Sim.getCurrentDrawAmps( )));
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public void initialize( )
  {
    double curCounts = 0.0;

    DataLogManager.log(getSubsystem( ) + ": subsystem initialized!");

    setGateHook(false);
    setClimberStopped( );

    if (m_validCL14)
      curCounts = m_motorCL14.getSelectedSensorPosition(0);

    m_curInches = countsToInches((int) curCounts);
    m_targetInches = m_curInches;
    DataLogManager.log(getSubsystem( ) + ": Init Target Inches: " + m_targetInches);
  }

  // Dump all Talon faults
  public void faultDump( )
  {
    PhoenixUtil.getInstance( ).talonFXFaultDump(m_motorCL14, "CL14");
    PhoenixUtil.getInstance( ).talonFXFaultDump(m_motorCL15, "CL15");
  }

  private static int elevatorRPMToNative(double rpm)
  {
    return (int) ((rpm * CLConsts.kClimberCPR) / (60.0 * 10.0)); // CTRE native units are (counts per 100ms)
  }

  private double inchesToCounts(double inches)
  {
    return inches / Constants.CLConsts.kInchesPerCount;
  }

  private double countsToInches(int counts)
  {
    return counts * Constants.CLConsts.kInchesPerCount;
  }

  private void climberTalonInitialize(WPI_TalonFX motor, boolean inverted)
  {
    motor.setInverted(inverted);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setInverted");
    motor.setNeutralMode(NeutralMode.Brake);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setNeutralMode");
    motor.setSafetyEnabled(false);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setSafetyEnabled");

    motor.configVoltageCompSaturation(12.0, 0);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configVoltageCompSaturation");
    motor.enableVoltageCompensation(true);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "enableVoltageCompensation");

    motor.configSupplyCurrentLimit(m_supplyCurrentLimits);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configSupplyCurrentLimits");
    motor.configStatorCurrentLimit(m_statorCurrentLimits);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configStatorCurrentLimits");

    // Configure sensor settings
    motor.setSelectedSensorPosition(0, 0, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "setSelectedSensorPosition");
    motor.configAllowableClosedloopError(0, m_CLAllowedError, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configAllowableClosedloopError");

    motor.configMotionCruiseVelocity(m_velocity, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configMotionCruiseVelocity");
    motor.configMotionAcceleration(m_acceleration, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configMotionAcceleration");
    motor.configMotionSCurveStrength(m_sCurveStrength, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "configMotionSCurveStrength");

    // Configure Magic Motion settings
    motor.config_kF(0, m_pidKf, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "config_kF");
    motor.config_kP(0, m_pidKp, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "config_kP");
    motor.config_kI(0, m_pidKi, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "config_kI");
    motor.config_kD(0, m_pidKd, CANTIMEOUT);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "config_kD");
    motor.selectProfileSlot(SLOTINDEX, PIDINDEX);
    PhoenixUtil.getInstance( ).checkTalonError(motor, "selectProfileSlot");

    motor.set(ControlMode.PercentOutput, 0.0);
  }

  public void moveClimberWithJoysticks(XboxController joystick)
  {

    double yCLValue = 0.0;
    double motorOutput = 0.0;
    double manualSpeedMax = 0.3;

    yCLValue = -joystick.getLeftY( );
    if (yCLValue > -0.1 && yCLValue < 0.1)
    {
      if (state != Constants.CLConsts.CLMode.CLIMBER_STOPPED)
        DataLogManager.log(getSubsystem( ) + "CL Climber Stopped");
      state = Constants.CLConsts.CLMode.CLIMBER_STOPPED;
    }
    else
    {
      // If joystick is above a value, climber will move up
      if (yCLValue > m_stickDeadband)
      {
        if (state != Constants.CLConsts.CLMode.CLIMBER_UP)
          DataLogManager.log(getSubsystem( ) + ("CL Climber Up"));
        state = Constants.CLConsts.CLMode.CLIMBER_UP;

        yCLValue -= m_stickDeadband;
        yCLValue *= (1.0 / (1.0 - m_stickDeadband));
        motorOutput = manualSpeedMax * (yCLValue * Math.abs(yCLValue));
      }
      // If joystick is below a value, climber will move down
      else if (yCLValue < -m_stickDeadband)
      {
        if (state != Constants.CLConsts.CLMode.CLIMBER_DOWN)
          DataLogManager.log(getSubsystem( ) + "CL Climber Down");
        state = Constants.CLConsts.CLMode.CLIMBER_DOWN;

        yCLValue += m_stickDeadband;
        yCLValue *= (1.0 / (1.0 - m_stickDeadband));
        motorOutput = manualSpeedMax * (yCLValue * Math.abs(yCLValue));
      }
    }

    if (m_validCL14)
      m_motorCL14.set(ControlMode.PercentOutput, motorOutput);

    if (m_validCL15)
      m_motorCL15.set(ControlMode.PercentOutput, motorOutput);
  }

  public void setClimberStopped( )
  {
    DataLogManager.log(getSubsystem( ) + ": CL Set Climber Stopped");

    if (m_validCL14)
      m_motorCL14.set(ControlMode.PercentOutput, 0);

    if (m_validCL15)
      m_motorCL15.set(ControlMode.PercentOutput, 0);
  }

  public void setGateHook(boolean hookClosed)
  {
    if (hookClosed != m_gateHook.get( ))
    {
      DataLogManager.log(getSubsystem( ) + ": CL HOOK - " + ((hookClosed) ? "OPEN" : "CLOSED"));
      SmartDashboard.putBoolean("CL_hookClosed", hookClosed);

      m_gateHook.set(hookClosed);
    }
  }

  public void moveToCalibrate( )
  {
    double motorCalibrateSpeed = -0.1;

    if (m_validCL14)
      m_motorCL14.set(ControlMode.PercentOutput, motorCalibrateSpeed);

    if (m_validCL15)
      m_motorCL15.set(ControlMode.PercentOutput, motorCalibrateSpeed);
  }

  public void calibrate( )
  {
    if (m_validCL14)
      m_motorCL14.setSelectedSensorPosition(0, 0, CANTIMEOUT);

    if (m_validCL15)
      m_motorCL15.setSelectedSensorPosition(0, 0, CANTIMEOUT);

    m_targetInches = 0;
    m_curInches = 0;
    m_calibrated = true;
    SmartDashboard.putBoolean("CL_calibrated", m_calibrated);
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

  public void moveClimberDistanceInit(Height state)
  {
    if (m_climberDebug != 0)
    {
      m_velocity = (int) SmartDashboard.getNumber("CL_velocity", m_velocity);
      m_acceleration = (int) SmartDashboard.getNumber("CL_acceleration", m_acceleration);
      m_sCurveStrength = (int) SmartDashboard.getNumber("CL_sCurveStrength", m_sCurveStrength);
      m_pidKf = SmartDashboard.getNumber("CL_pidKf", m_pidKf);
      m_pidKp = SmartDashboard.getNumber("CL_pidKp", m_pidKp);
      m_pidKi = SmartDashboard.getNumber("CL_pidKi", m_pidKi);
      m_pidKd = SmartDashboard.getNumber("CL_pidKd", m_pidKd);

      m_motorCL14.configMotionCruiseVelocity(m_velocity, 0);
      m_motorCL14.configMotionAcceleration(m_acceleration, 0);
      m_motorCL14.configMotionSCurveStrength(m_sCurveStrength, 0);
      m_motorCL14.config_kF(0, m_pidKf, 0);
      m_motorCL14.config_kP(0, m_pidKp, 0);
      m_motorCL14.config_kI(0, m_pidKi, 0);
      m_motorCL14.config_kD(0, m_pidKd, 0);

      m_motorCL15.configMotionCruiseVelocity(m_velocity, 0);
      m_motorCL15.configMotionAcceleration(m_acceleration, 0);
      m_motorCL15.configMotionSCurveStrength(m_sCurveStrength, 0);
      m_motorCL15.config_kF(0, m_pidKf, 0);
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
        m_targetInches = SmartDashboard.getNumber("CL_stowHeight", m_stowHeight);
        break;
      case EXTEND_L2_HEIGHT :
        m_targetInches = SmartDashboard.getNumber("CL_extendL2", m_extendL2);
        break;
      case ROTATE_L3_HEIGHT :
        m_targetInches = SmartDashboard.getNumber("CL_rotateL3", m_rotateL3);
        break;
      case GATEHOOK_REST_HEIGHT :
        m_targetInches = SmartDashboard.getNumber("CL_gatehookRestHeight", m_gatehookRestHeight);
        break;
      case RAISE_L4_HEIGHT :
        m_targetInches = SmartDashboard.getNumber("CL_raiseL4", m_raiseL4);
        break;
      default :
        DataLogManager.log(getSubsystem( ) + ": requested height is invalid - " + state);
        return;
    }
  }

  public void moveClimberDistanceIsFinished( )
  {

  }
}
