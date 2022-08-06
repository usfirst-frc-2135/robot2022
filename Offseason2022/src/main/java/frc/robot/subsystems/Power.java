
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 *
 */
public class Power extends SubsystemBase
{
  private PowerDistribution powerDistribution;

  /**
   *
   */
  public Power( )
  {
    setName("Power");
    powerDistribution = new PowerDistribution( );
    addChild("PowerDistribution", powerDistribution);
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

  public void initialize( )
  {
    // TODO: round getVoltage()
    DataLogManager.log(getSubsystem( ) + "Init - Voltage :" + powerDistribution.getVoltage( ));
  }

  public void FaultDump( )
  {
    DataLogManager.log(getSubsystem( ) + "Temperature :" + powerDistribution.getTemperature( ));
    DataLogManager.log(getSubsystem( ) + "Input Voltage : " + powerDistribution.getVoltage( ));
    for (int i = 0; i <= 15; i++)
    {
      DataLogManager.log(getSubsystem( ) + "Chan :" + i + "Current :" + powerDistribution.getCurrent(i));
    }
    DataLogManager.log(getSubsystem( ) + "Total Current : " + powerDistribution.getTotalCurrent( ));
    DataLogManager.log(getSubsystem( ) + "Total Power : " + powerDistribution.getTotalPower( ));
    DataLogManager.log(getSubsystem( ) + "Total Energy : " + powerDistribution.getTotalEnergy( ));

    powerDistribution.resetTotalEnergy( );
    powerDistribution.clearStickyFaults( );
  }

}
