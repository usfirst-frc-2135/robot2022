
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 *
 */
public class Drivetrain extends SubsystemBase
{
  private WPI_TalonFX driveL1;
  private WPI_TalonFX driveL2;
  private WPI_TalonFX driveR3;
  private WPI_TalonFX driveR4;
  private Pigeon2     pigeonIMU;

  /**
   *
   */
  public Drivetrain( )
  {
    driveL1 = new WPI_TalonFX(1);

    driveL2 = new WPI_TalonFX(2);

    driveR3 = new WPI_TalonFX(3);

    driveR4 = new WPI_TalonFX(4);

    pigeonIMU = new Pigeon2(0);
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
