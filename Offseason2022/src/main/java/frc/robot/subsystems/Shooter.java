
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Falcon500;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.Constants.SHConsts;
import frc.robot.Constants.SHConsts.Mode;
import frc.robot.RobotContainer;
import frc.robot.frc2135.PhoenixUtil;
import frc.robot.frc2135.RobotConfig;

/**
 *
 */
public class Shooter extends SubsystemBase
{
  // Constants
  private static final int                CANTIMEOUT            = 30;  // CAN timeout in msec
  private static final int                PIDINDEX              = 0;   // PID in use (0-primary, 1-aux)
  private static final int                SLOTINDEX             = 0;   // Use first PID slot

  // Devices and simulation objects
  private ErrorCode                       error;
  private WPI_TalonFX                     m_motorSH11           = new WPI_TalonFX(SHConsts.kCANID);
  private TalonFXSimCollection            m_motorSim            = new TalonFXSimCollection(m_motorSH11);
  private FlywheelSim                     m_flywheelSim         =
      new FlywheelSim(DCMotor.getFalcon500(1), SHConsts.kFlywheelGearRatio, 0.01);
  private LinearFilter                    m_flywheelFilter      = LinearFilter.singlePoleIIR(0.1, 0.02);

  private SupplyCurrentLimitConfiguration m_supplyCurrentLimits = new SupplyCurrentLimitConfiguration(true,
      Falcon500.kSupplyCurrentLimit, Falcon500.kSupplyTriggerCurrent, Falcon500.kSupplyTriggerTime);
  private StatorCurrentLimitConfiguration m_statorCurrentLimits = new StatorCurrentLimitConfiguration(true,
      Falcon500.kStatorCurrentLimit, Falcon500.kStatorTriggerCurrent, Falcon500.kStatorTriggerTime);

  // Declare module variables
  private boolean                         m_validSH11           = false; // Health indicator for shooter talon 11
  private int                             m_resetCountSH11      = 0;     // reset counter for motor
  private boolean                         m_ifShooterTest       = false; // checks to see if testing the shooter
  private boolean                         m_atDesiredSpeed      = false; // Indicates flywheel RPM is close to target
  private boolean                         m_atDesiredSpeedPrevious;

  private double                          m_flywheelTargetRPM;      // Requested flywheel RPM
  private double                          m_flywheelRPM;            // Current flywheel RPM
  private Mode                            m_curMode;                // Current shooter mode

  // Configuration file parameters
  private double                          m_flywheelPidKf;           // Flywheel PID force constant
  private double                          m_flywheelPidKp;           // Flywheel PID proportional constant
  private double                          m_flywheelPidKi;           // Flywheel PID integral constant
  private double                          m_flywheelPidKd;           // Flywheel PID derivative constant
  private double                          m_flywheelNeutralDeadband; // Flywheel PID neutral deadband in percent

  private double                          m_flywheelPrimeRPM;        // Flywheel RPM for priming before shooting
  private double                          m_flywheelLowerTargetRPM;  // Flywheel target RPM for shooting lower hub
  private double                          m_flywheelUpperTargetRPM;  // Flywheel target RPM for shooting upper hub
  private double                          m_flywheelToleranceRPM;    // Allowed variation from target RPM

  /**
   *
   */
  public Shooter( )
  {
    // Set the names for this subsystem for later use
    setName("Shooter");
    setSubsystem("Shooter");

    // Confirm the motor controller is talking and initialize it to factory defaults
    m_validSH11 = PhoenixUtil.getInstance( ).talonFXInitialize(m_motorSH11, "SH11");
    SmartDashboard.putBoolean("HL_validSH11", m_validSH11);

    // Get values from config file
    RobotConfig config = RobotConfig.getInstance( );
    m_flywheelPidKf = config.getValueAsDouble("SH_flywheelPidKf", SHConsts.kFlywheelPidKf);
    m_flywheelPidKp = config.getValueAsDouble("SH_flywheelPidKp", SHConsts.kFlywheelPidKp);
    m_flywheelPidKi = config.getValueAsDouble("SH_flywheelPidKi", SHConsts.kFlywheelPidKi);
    m_flywheelPidKd = config.getValueAsDouble("SH_flywheelPidKd", SHConsts.kFlywheelPidKd);
    m_flywheelNeutralDeadband = config.getValueAsDouble("SH_flywheelNeutralDeadband", SHConsts.kFlywheelNeutralDeadband);

    m_flywheelToleranceRPM = config.getValueAsDouble("SH_flywheelToleranceRPM", SHConsts.kFlywheelToleranceRPM);
    m_flywheelPrimeRPM = config.getValueAsDouble("SH_flywheelLowerTargetRPM", SHConsts.kFlywheelPrimeRPM);
    m_flywheelLowerTargetRPM = config.getValueAsDouble("SH_flywheelLowerTargetRPM", SHConsts.kFlywheelLowerTargetRPM);
    m_flywheelUpperTargetRPM = config.getValueAsDouble("SH_flywheelUpperTargetRPM", SHConsts.kFlywheelUpperTargetRPM);

    // Put config file values to smart dashboard
    SmartDashboard.putNumber("SH_flywheelPidKf", m_flywheelPidKf);
    SmartDashboard.putNumber("SH_flywheelPidKp", m_flywheelPidKp);
    SmartDashboard.putNumber("SH_flywheelPidKi", m_flywheelPidKi);
    SmartDashboard.putNumber("SH_flywheelPidKd", m_flywheelPidKd);
    SmartDashboard.putNumber("SH_flywheelNeutralDeadband", m_flywheelNeutralDeadband);

    SmartDashboard.putNumber("SH_flywheelLowerTargetRPM", m_flywheelPrimeRPM);
    SmartDashboard.putNumber("SH_flywheelLowerTargetRPM", m_flywheelLowerTargetRPM);
    SmartDashboard.putNumber("SH_flywheelUpperTargetRPM", m_flywheelUpperTargetRPM);
    SmartDashboard.putNumber("SH_flywheelToleranceRPM", m_flywheelToleranceRPM);

    // Initialize the motor controller for use as a shooter
    if (m_validSH11)
    {
      // Set motor direction and coast/brake mode
      m_motorSH11.setInverted(true);
      PhoenixUtil.getInstance( ).checkError(m_motorSH11.getLastError( ), "setInverted");
      m_motorSH11.setNeutralMode(NeutralMode.Coast);
      PhoenixUtil.getInstance( ).checkError(m_motorSH11.getLastError( ), "setNeutralMode");
      m_motorSH11.setSafetyEnabled(false);
      PhoenixUtil.getInstance( ).checkError(m_motorSH11.getLastError( ), "setSafetyEnabled");

      // Enable voltage compensation
      m_motorSH11.configNeutralDeadband(m_flywheelNeutralDeadband, CANTIMEOUT);
      PhoenixUtil.getInstance( ).checkError(m_motorSH11.getLastError( ), "configNeutralDeadband");
      m_motorSH11.configPeakOutputReverse(0.0, CANTIMEOUT);
      PhoenixUtil.getInstance( ).checkError(m_motorSH11.getLastError( ), "configPeakOutputReverse");

      m_motorSH11.configSupplyCurrentLimit(m_supplyCurrentLimits);
      PhoenixUtil.getInstance( ).checkError(m_motorSH11.getLastError( ), "configSupplyCurrentLimits");
      m_motorSH11.configStatorCurrentLimit(m_statorCurrentLimits);
      PhoenixUtil.getInstance( ).checkError(m_motorSH11.getLastError( ), "configStatorCurrentLimits");

      // Configure sensor settings
      m_motorSH11.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PIDINDEX, CANTIMEOUT);
      PhoenixUtil.getInstance( ).checkError(m_motorSH11.getLastError( ), "configSelectedFeedbackSensor");
      m_motorSH11.setSensorPhase(true);
      PhoenixUtil.getInstance( ).checkError(m_motorSH11.getLastError( ), "setSensorPhase");

      configFlywheelPid(CANTIMEOUT);

      m_motorSH11.set(ControlMode.Velocity, 0.0);
      PhoenixUtil.getInstance( ).checkError(m_motorSH11.getLastError( ), "set 0.0");
    }

    // Initialize subsystem settings for current states
    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This periodic method will be called once per scheduler run

    RobotContainer rc = RobotContainer.getInstance( );

    // Calculate flywheel RPM and display on dashboard
    // if (m_validSH11)
    m_flywheelRPM = m_flywheelFilter.calculate(flywheelNativeToRPM((m_motorSH11.getSelectedSensorVelocity(PIDINDEX))));
    SmartDashboard.putNumber("SH_flywheelRPM", m_flywheelRPM);

    m_atDesiredSpeed = Math.abs(m_flywheelTargetRPM - m_flywheelRPM) < m_flywheelToleranceRPM;
    SmartDashboard.putBoolean("SH_atDesiredSpeed", m_atDesiredSpeed);

    if (m_atDesiredSpeed != m_atDesiredSpeedPrevious)
    {
      DataLogManager.log(getSubsystem( ) + ": at desired speed now " + m_flywheelTargetRPM);
      m_atDesiredSpeedPrevious = m_atDesiredSpeed;
    }

    // Control CANdle LEDs based on shooter status
    if (m_curMode != Mode.SHOOTER_STOP)
    {
      if (!m_atDesiredSpeed)
      {
        rc.m_led.setColor(LEDColor.LEDCOLOR_BLUE);
        DataLogManager.log(getSubsystem( ) + ": m_flywheelRPM " + m_flywheelRPM);
      }
      else
        rc.m_led.setColor(LEDColor.LEDCOLOR_GREEN);
    }
    else
      rc.m_led.setColor(LEDColor.LEDCOLOR_OFF);

    // Display motor current on dashboard
    double currentSH11 = 0.0;

    if (m_validSH11)
      currentSH11 = m_motorSH11.getStatorCurrent( );
    SmartDashboard.putNumber("SH_currentSH11", currentSH11);

    // Count motor controller resets and display on dashboard
    if (m_motorSH11.hasResetOccurred( ))
      SmartDashboard.putNumber("HL_resetCountSH11", ++m_resetCountSH11);
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run when in simulation

    // Set input flywheel voltage from the motor setting
    m_motorSim.setBusVoltage(RobotController.getInputVoltage( ));
    m_flywheelSim.setInput(m_motorSim.getMotorOutputLeadVoltage( ));

    // update for 20 msec loop
    m_flywheelSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_motorSim.setIntegratedSensorVelocity(flywheelRPMToNative(m_flywheelSim.getAngularVelocityRPM( )));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_flywheelSim.getCurrentDrawAmps( )));
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // private static methods first

  private static int flywheelRPMToNative(double rpm)
  {
    return (int) ((rpm * SHConsts.kFlywheelCPR) / (60.0 * 10.0)); // CTRE native units are (counts per 100ms)
  }

  private static double flywheelNativeToRPM(double nativeUnits)
  {
    return (nativeUnits * 60.0 * 10.0) / SHConsts.kFlywheelCPR; // CTRE native units are (counts per 100ms)
  }

  private void configFlywheelPid(int timeout)
  {
    // Configure velocity PIDF settings
    m_motorSH11.config_kF(SLOTINDEX, m_flywheelPidKf, timeout);
    PhoenixUtil.getInstance( ).checkError(m_motorSH11.getLastError( ), "config_kF");
    m_motorSH11.config_kP(SLOTINDEX, m_flywheelPidKp, timeout);
    PhoenixUtil.getInstance( ).checkError(m_motorSH11.getLastError( ), "config_kP");
    m_motorSH11.config_kI(SLOTINDEX, m_flywheelPidKi, timeout);
    PhoenixUtil.getInstance( ).checkError(m_motorSH11.getLastError( ), "config_kI");
    m_motorSH11.config_kD(SLOTINDEX, m_flywheelPidKd, timeout);
    PhoenixUtil.getInstance( ).checkError(m_motorSH11.getLastError( ), "config_kD");
    m_motorSH11.selectProfileSlot(SLOTINDEX, PIDINDEX);
    PhoenixUtil.getInstance( ).checkError(m_motorSH11.getLastError( ), "selectProfileSlot");
  }

  // public methods

  public void initialize( )
  {
    DataLogManager.log(getSubsystem( ) + ": subsystem initialized!");
    setShooterMode(Mode.SHOOTER_STOP);
  }

  public void dumpFaults( )
  {
    if (m_validSH11)
      PhoenixUtil.getInstance( ).talonFXFaultDump(m_motorSH11, "SH11");
  }

  public void setShooterMode(Mode mode)
  {
    m_curMode = mode;

    DataLogManager.log(getSubsystem( ) + ": set shooter mode " + mode);

    // Get latest flywheel settings from dashboard
    m_flywheelLowerTargetRPM = SmartDashboard.getNumber("SH_flywheelLowerTargetRPM", m_flywheelLowerTargetRPM);
    m_flywheelPrimeRPM = SmartDashboard.getNumber("SH_flywheelLowerTargetRPM", m_flywheelPrimeRPM);
    m_flywheelUpperTargetRPM = SmartDashboard.getNumber("SH_flywheelUpperTargetRPM", m_flywheelUpperTargetRPM);
    m_flywheelToleranceRPM = SmartDashboard.getNumber("SH_flywheelToleranceRPM", m_flywheelToleranceRPM);

    // If in shooter test mode, get PIDF settings and program motor controller
    if (m_validSH11 && m_ifShooterTest)
    {
      m_flywheelPidKf = SmartDashboard.getNumber("SH_flywheelPidKf", m_flywheelPidKf);
      m_flywheelPidKp = SmartDashboard.getNumber("SH_flywheelPidKp", m_flywheelPidKp);
      m_flywheelPidKi = SmartDashboard.getNumber("SH_flywheelPidKi", m_flywheelPidKi);
      m_flywheelPidKd = SmartDashboard.getNumber("SH_flywheelPidKd", m_flywheelPidKd);

      configFlywheelPid(0);

      DataLogManager.log(getSubsystem( ) + ": kF " + m_flywheelPidKf + " kP " + m_flywheelPidKp + " kI " + m_flywheelPidKi + " kD"
          + m_flywheelPidKd);
    }

    // Select the shooter RPM from the requested mode
    switch (mode)
    {
      case SHOOTER_REVERSE :
        m_flywheelTargetRPM = SHConsts.kFlywheelReverseRPM;
        break;
      case SHOOTER_STOP :
        m_flywheelTargetRPM = 0.0;
        break;
      case SHOOTER_PRIME :
        m_flywheelTargetRPM = m_flywheelPrimeRPM;
        break;
      case SHOOTER_LOWERHUB :
        m_flywheelTargetRPM = m_flywheelLowerTargetRPM;
        break;
      case SHOOTER_UPPERHUB :
        m_flywheelTargetRPM = m_flywheelUpperTargetRPM;
        break;
      default :
        DataLogManager.log(getSubsystem( ) + ": invalid shooter mode requested " + mode);
        break;
    }

    if (m_validSH11)
      m_motorSH11.set(ControlMode.Velocity, flywheelRPMToNative(m_flywheelTargetRPM));

    DataLogManager.log(getSubsystem( ) + ": target speed is " + m_flywheelTargetRPM);
  }

  public boolean isAtDesiredSpeed( )
  {

    return m_atDesiredSpeed;
  }

  public void setReverseInit( )
  {
    setShooterMode(Mode.SHOOTER_STOP);
  }

  public void setReverseExecute( )
  {
    if (m_flywheelRPM < SHConsts.kReverseRPMThreshold)
    {
      m_motorSH11.configPeakOutputReverse(-1.0);
      DataLogManager.log(getSubsystem( ) + ": reverse mode now available");
      setShooterMode(Mode.SHOOTER_REVERSE);
    }
  }

  public void setReverseEnd( )
  {
    m_motorSH11.configPeakOutputForward(0.0);
    setShooterMode(Mode.SHOOTER_STOP);
  }
}
