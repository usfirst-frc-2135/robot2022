
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

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
import frc.robot.Constants;
import frc.robot.Constants.Shooter.Mode;
import frc.robot.frc2135.PhoenixUtil;
import frc.robot.frc2135.RobotConfig;

/**
 *
 */
public class Shooter extends SubsystemBase
{
  // Constants
  private static final int     PIDINDEX       = 0;   // PID in use (0-primary, 1-aux)
  private static final int     SLOTINDEX      = 0;   // Use first PID slot
  private static final int     CANTIMEOUT     = 30;  // CAN timeout in msec

  // Devices
  private WPI_TalonFX          motorSH11      = new WPI_TalonFX(11);
  private TalonFXSimCollection motorSim       = new TalonFXSimCollection(motorSH11);
  private FlywheelSim          flywheelSim    = new FlywheelSim(DCMotor.getFalcon500(1),
      Constants.Shooter.kFlywheelGearRatio, 0.01);
  private LinearFilter         flywheelFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

  // Configuration file parameters
  private double               flywheelPidKf;           // Flywheel PID force constant
  private double               flywheelPidKp;           // Flywheel PID proportional constant
  private double               flywheelPidKi;           // Flywheel PID integral constant
  private double               flywheelPidKd;           // Flywheel PID derivative constant
  private double               flywheelNeutralDeadband; // Flywheel PID neutral deadband in percent
  private double               flywheelLowerTargetRPM;  // Target flywheel RPM for shooting lower hub
  private double               flywheelUpperTargetRPM;  // Target flywheel RPM for shooting upper hub
  private double               flywheelToleranceRPM;    // Allowed variation from target RPM

  // Declare module variables
  private boolean              SH11Valid      = false; // Health indicator for shooter talon 11
  private int                  resetCountSH11 = 0;     // reset counter for motor
  private boolean              ifShooterTest  = false; // checks to see if testing the shooter

  private double               flywheelRPM;            // Current flywheel RPM
  private Mode                 curMode;                // Current shooter mode

  /**
   *
   */
  public Shooter( )
  {
    setName("Shooter");
    setSubsystem("Shooter");

    motorSH11 = new WPI_TalonFX(11);

    SupplyCurrentLimitConfiguration supplyCurrentLimits = new SupplyCurrentLimitConfiguration(true, 45.0, 45.0, 0.001);
    StatorCurrentLimitConfiguration statorCurrentLimits = new StatorCurrentLimitConfiguration(true, 80.0, 80.0, 0.001);

    SH11Valid = PhoenixUtil.getInstance( ).talonFXInitialize(motorSH11, "SH11");
    SmartDashboard.putBoolean("HL_SH11Valid", SH11Valid);

    // Get values from config file
    RobotConfig config = RobotConfig.getInstance( );
    flywheelPidKf = config.getValueAsDouble("SH_FlywheelPidKf", 0.0475);
    flywheelPidKp = config.getValueAsDouble("SH_FlywheelPidKf", 0.05);
    flywheelPidKi = config.getValueAsDouble("SH_FlywheelPidKf", 0.0);
    flywheelPidKd = config.getValueAsDouble("SH_FlywheelPidKf", 0.0);
    flywheelNeutralDeadband = config.getValueAsDouble("SH_FlywheelPidKf", 0.004);
    flywheelLowerTargetRPM = config.getValueAsDouble("SH_FlywheelPidKf", 1450.0);
    flywheelUpperTargetRPM = config.getValueAsDouble("SH_FlywheelPidKf", 3000.0);
    flywheelToleranceRPM = config.getValueAsDouble("SH_FlywheelPidKf", Constants.Shooter.kFlywheelPrimeRPM);

    // Put values to smart dashboard
    SmartDashboard.putNumber("SH_flywheelPidKf", flywheelPidKf);
    SmartDashboard.putNumber("SH_flywheelPidKp", flywheelPidKp);
    SmartDashboard.putNumber("SH_flywheelPidKi", flywheelPidKi);
    SmartDashboard.putNumber("SH_flywheelPidKd", flywheelPidKd);
    SmartDashboard.putNumber("SH_flywheelNeutralDeadband", flywheelNeutralDeadband);
    SmartDashboard.putNumber("SH_flywheelLowerTargetRPM", flywheelLowerTargetRPM);
    SmartDashboard.putNumber("SH_flywheelUpperTargetRPM", flywheelUpperTargetRPM);
    SmartDashboard.putNumber("SH_flywheelToleranceRPM", flywheelToleranceRPM);

    if (SH11Valid)
    {
      // Set motor direction and coast/brake mode
      motorSH11.setInverted(true);
      motorSH11.setNeutralMode(NeutralMode.Coast);
      motorSH11.setSafetyEnabled(false);

      // Enable voltage compensation
      motorSH11.configNeutralDeadband(flywheelNeutralDeadband, CANTIMEOUT);
      motorSH11.configPeakOutputReverse(0.0, CANTIMEOUT);

      motorSH11.configSupplyCurrentLimit(supplyCurrentLimits);
      motorSH11.configStatorCurrentLimit(statorCurrentLimits);

      // Configure sensor settings
      motorSH11.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PIDINDEX, CANTIMEOUT);
      motorSH11.setSensorPhase(true);

      // Configure velocity PIDF settings
      motorSH11.config_kF(SLOTINDEX, flywheelPidKf, CANTIMEOUT);
      motorSH11.config_kP(SLOTINDEX, flywheelPidKp, CANTIMEOUT);
      motorSH11.config_kI(SLOTINDEX, flywheelPidKi, CANTIMEOUT);
      motorSH11.config_kD(SLOTINDEX, flywheelPidKd, CANTIMEOUT);
      motorSH11.selectProfileSlot(SLOTINDEX, PIDINDEX);

      motorSH11.set(ControlMode.Velocity, 0.0);

      initialize( );
    }
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
    if (SH11Valid)
      flywheelRPM = flywheelFilter
          .calculate(flywheelNativeToRPM((motorSH11.getSelectedSensorVelocity(PIDINDEX))));

    SmartDashboard.putNumber("SH_flywheelRPM", flywheelRPM);

    if (curMode != Mode.SHOOTER_STOP)
    {
      if (!isAtDesiredSpeed( ))
      {
        ; // TODO: LED Set the color to BLUE
        DataLogManager.log(getSubsystem( ) + ": flywheelRPM " + flywheelRPM);
      }
      else
        ; // TODO: LED Set the color to GREEN
    }
    else
      ; // TODO: LED Set the color to OFF

    double currentSH11 = 0.0;

    if (SH11Valid)
      currentSH11 = motorSH11.getStatorCurrent( );

    SmartDashboard.putNumber("SH_currentSH11", currentSH11);

    if (motorSH11.hasResetOccurred( ))
      SmartDashboard.putNumber("HL_resetCountSH11", ++resetCountSH11);
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run when in simulation

    // Set input flywheel voltage from the motor setting
    motorSim.setBusVoltage(RobotController.getInputVoltage( ));
    flywheelSim.setInput(motorSim.getMotorOutputLeadVoltage( ));

    // update
    flywheelSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    motorSim.setIntegratedSensorVelocity(flywheelRPMToNative(flywheelSim.getAngularVelocityRPM( )));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps( )));
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static int flywheelRPMToNative(double rpm)
  {
    return (int) ((rpm * Constants.Shooter.kFlywheelCPR) / (60.0 * 10.0));
  }

  private static double flywheelNativeToRPM(double nativeUnits)
  {
    return (nativeUnits * 60.0 * 10.0) / Constants.Shooter.kFlywheelCPR;
  }

  public void initialize( )
  {
    DataLogManager.log(getSubsystem( ) + ": subsystem initialized!");
    setShooterMode(Mode.SHOOTER_STOP);
  }

  public void dumpFaults( )
  {
    if (SH11Valid)
      PhoenixUtil.getInstance( ).talonFXFaultDump(motorSH11, "SH11");
  }

  public void setShooterMode(Mode mode)
  {
    double flywheelRPM = 0.0;

    curMode = mode;

    DataLogManager.log(getSubsystem( ) + ": set shooter mode " + mode);

    flywheelLowerTargetRPM = SmartDashboard.getNumber("SH_flywheelLowerTargetRPM", flywheelLowerTargetRPM);
    flywheelUpperTargetRPM = SmartDashboard.getNumber("SH_flywheelUpperTargetRPM", flywheelUpperTargetRPM);
    flywheelToleranceRPM = SmartDashboard.getNumber("SH_flywheelToleranceRPM", flywheelToleranceRPM);

    if (SH11Valid && ifShooterTest)
    {
      flywheelPidKf = SmartDashboard.getNumber("SH_flywheelPidKf", flywheelPidKf);
      flywheelPidKp = SmartDashboard.getNumber("SH_flywheelPidKp", flywheelPidKp);
      flywheelPidKi = SmartDashboard.getNumber("SH_flywheelPidKi", flywheelPidKi);
      flywheelPidKd = SmartDashboard.getNumber("SH_flywheelPidKd", flywheelPidKd);

      motorSH11.config_kF(SLOTINDEX, flywheelPidKf, 0);
      motorSH11.config_kP(SLOTINDEX, flywheelPidKp, 0);
      motorSH11.config_kI(SLOTINDEX, flywheelPidKi, 0);
      motorSH11.config_kD(SLOTINDEX, flywheelPidKd, 0);
      motorSH11.selectProfileSlot(SLOTINDEX, PIDINDEX);

      DataLogManager
          .log(getSubsystem( ) + ": kF " + flywheelPidKf + " kP " + flywheelPidKp + " kI " + flywheelPidKi + " kD");
    }

    switch (mode)
    {
      case SHOOTER_REVERSE :
        flywheelRPM = -1000;
        break;
      case SHOOTER_STOP :
        flywheelRPM = 0;
        break;
      case SHOOTER_PRIME :
        flywheelRPM = Constants.Shooter.kFlywheelPrimeRPM;
        break;
      case SHOOTER_LOWERHUB :
        flywheelRPM = flywheelLowerTargetRPM;
        break;
      case SHOOTER_UPPERHUB :
        flywheelRPM = flywheelUpperTargetRPM;
        break;
      default :
        DataLogManager.log(getSubsystem( ) + ": invalid shooter mode requested " + mode);
        break;
    }

    if (SH11Valid)
      motorSH11.set(ControlMode.Velocity, flywheelRPMToNative(flywheelRPM));

    DataLogManager.log(getSubsystem( ) + ": target speed is " + flywheelRPM);
  }

  private static boolean atDesiredSpeedPrevious;

  public boolean isAtDesiredSpeed( )
  {
    double desiredSpeed;
    boolean atDesiredSpeed;

    switch (curMode)
    {
      default :
      case SHOOTER_STOP :
        desiredSpeed = 0.0;
        break;
      case SHOOTER_PRIME :
        desiredSpeed = Constants.Shooter.kFlywheelPrimeRPM;
        break;
      case SHOOTER_LOWERHUB :
        desiredSpeed = flywheelLowerTargetRPM;
        break;
      case SHOOTER_UPPERHUB :
        desiredSpeed = flywheelUpperTargetRPM;
        break;
      case SHOOTER_REVERSE :
        desiredSpeed = 0.0;
        break;
    }

    atDesiredSpeed = (Math.abs(desiredSpeed - flywheelRPM) < flywheelToleranceRPM);

    if (atDesiredSpeed != atDesiredSpeedPrevious)
    {
      DataLogManager.log(getSubsystem( ) + ": at desired speed now " + atDesiredSpeed);
      atDesiredSpeedPrevious = atDesiredSpeed;
    }

    return atDesiredSpeed;
  }

  public void setReverseInit( )
  {
    setShooterMode(Mode.SHOOTER_STOP);
  }

  public void setReverseExecute( )
  {
    double reverseRPMThreshold = 20;

    if (flywheelRPM < reverseRPMThreshold)
    {
      motorSH11.configPeakOutputReverse(-1.0);
      DataLogManager.log(getSubsystem( ) + ": reverse mode now available");
      setShooterMode(Mode.SHOOTER_REVERSE);
    }
  }

  public void setReverseEnd( )
  {
    motorSH11.configPeakOutputForward(0.0);
    setShooterMode(Mode.SHOOTER_STOP);
  }
}
