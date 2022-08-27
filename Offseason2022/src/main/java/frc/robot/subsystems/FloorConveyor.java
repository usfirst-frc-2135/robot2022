
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FCConsts;

/**
 *
 */
public class FloorConveyor extends SubsystemBase
{
  private WPI_TalonFX                     m_motorFC8            = new WPI_TalonFX(FCConsts.kCANID);

  private SupplyCurrentLimitConfiguration m_supplyCurrentLimits = new SupplyCurrentLimitConfiguration(true, 45.0, 45.0, 0.001);
  private StatorCurrentLimitConfiguration m_statorCurrentLimits = new StatorCurrentLimitConfiguration(true, 80.0, 80.0, 0.001);

  /**
   *
   */
  public FloorConveyor( )
  {
    // Set the names for this subsystem for later use
    setName("FloorConveyor");
    setSubsystem("FloorConveyor");
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
