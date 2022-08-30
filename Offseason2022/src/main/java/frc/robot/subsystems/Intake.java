
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.INConsts.INMode;

/**
 *
 */
public class Intake extends SubsystemBase
{
  // Constants
  // private static final int CANTIMEOUT = 30; // CAN timeout in msec

  // Devices and simulation objects (intake was moved from CAN bus to PWM1)
  // private WPI_TalonFX m_motorIN8 = new WPI_TalonFX(INConsts.kIN8CANID);
  private PWMTalonFX m_motorIN8 = new PWMTalonFX(INConsts.kINPWM1);
  private Solenoid   m_arm      = new Solenoid(0, PneumaticsModuleType.CTREPCM, INConsts.kArmSolenoid);

  // private SupplyCurrentLimitConfiguration m_supplyCurrentLimits = new
  // SupplyCurrentLimitConfiguration(true, 45.0, 45.0, 0.001);
  // private StatorCurrentLimitConfiguration m_statorCurrentLimits = new
  // StatorCurrentLimitConfiguration(true, 80.0, 80.0, 0.001);

  // Declare module variables
  private boolean    m_validIN8 = false;
  private double     m_acquireSpeed;
  private double     m_expelSpeed;

  /**
   *
   */
  public Intake( )
  {
    setName("Intake");
    setSubsystem("Intake");
    addChild("Arm", m_arm);

    SmartDashboard.putBoolean("HL_validIN8", m_validIN8);

    // Check if solenoid is functional or blacklisted
    DataLogManager.log(getSubsystem( ) + ": Arm Solenoid is " + ((m_arm.isDisabled( )) ? "BLACKLISTED" : "OK"));

    m_acquireSpeed = INConsts.kINAcquireSpeed;
    m_expelSpeed = INConsts.kINExpelSpeed;

    m_motorIN8.setInverted(true);
    m_motorIN8.setSafetyEnabled(false);
    m_motorIN8.set(0.0);

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

  // Put methods for controlling this subsystem here. Call these from Commands.

  public void initialize( )
  {
    DataLogManager.log(getSubsystem( ) + ": subsystem initialized!");
    setIntakeSpeed(INMode.INTAKE_STOP);
    setArmSolenoid(false);
  }

  public void setIntakeSpeed(INMode mode)
  {
    final String strName;
    double output = 0.0; // Default: off

    switch (mode)
    {
      default :
      case INTAKE_STOP :
        strName = "STOP";
        output = 0.0;
        break;
      case INTAKE_ACQUIRE :
        strName = "ACQUIRE";
        output = m_acquireSpeed;
        break;
      case INTAKE_EXPEL :
        strName = "EXPEL";
        output = m_expelSpeed;
        break;
    }

    // Set speed of intake and the percent output
    DataLogManager.log(getSubsystem( ) + ": Set Speed - " + strName);
    m_motorIN8.set(output);
  }

  public void setArmSolenoid(boolean extend)
  {
    DataLogManager.log(getSubsystem( ) + ": Arm " + ((extend) ? "DEPLOYED" : "STOWED"));
    SmartDashboard.putBoolean("IN_Deployed", extend);
    m_arm.set(extend);
  }
}
