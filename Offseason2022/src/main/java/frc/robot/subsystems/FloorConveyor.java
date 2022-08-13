
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 *
 */
public class FloorConveyor extends SubsystemBase
{
  private WPI_TalonFX motorFC8;

  /**
   *
   */
  public FloorConveyor( )
  {
    motorFC8 = new WPI_TalonFX(8);
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
