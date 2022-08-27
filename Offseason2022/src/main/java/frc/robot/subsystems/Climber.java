
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CLConsts;

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
  private WPI_TalonFX                     m_motorCL14           = new WPI_TalonFX(CLConsts.kLeftCANID);
  private WPI_TalonFX                     m_motorCL15           = new WPI_TalonFX(CLConsts.kRightCANID);
  private Solenoid                        m_gateHook            =
      new Solenoid(0, PneumaticsModuleType.CTREPCM, CLConsts.kGateHookSolenod);
  private CANCoder                        m_gateHookAngle       = new CANCoder(CLConsts.kCancoderID);
  private DigitalInput                    m_downLimitLeft       = new DigitalInput(CLConsts.kLeftLimitDIO);
  private DigitalInput                    m_downLimitRight      = new DigitalInput(CLConsts.kRightLimitDIO);

  private SupplyCurrentLimitConfiguration m_supplyCurrentLimits = new SupplyCurrentLimitConfiguration(true, 45.0, 45.0, 0.001);
  private StatorCurrentLimitConfiguration m_statorCurrentLimits = new StatorCurrentLimitConfiguration(true, 80.0, 80.0, 0.001);

  // Declare module variables

  /**
   *
   */
  public Climber( )
  {
    // Set the names for this subsystem for later use
    setName("Climber");
    setSubsystem("Climber");
    addChild("GateHook", m_gateHook);
    addChild("DownLimitLeft", m_downLimitLeft);
    addChild("DownLimitRight", m_downLimitRight);
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
}
