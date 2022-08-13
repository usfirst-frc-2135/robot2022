
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
  private PowerDistribution powerDistribution = new PowerDistribution( );;

  /**
   *
   */
  public Power( )
  {
    setName("Power");
    addChild("PowerDistribution", powerDistribution);

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

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public void initialize( )
  {
    // TODO: round getVoltage()
    DataLogManager.log(getSubsystem( ) + ": Init Voltage is " + String.format("%.1f", powerDistribution.getVoltage( )));
  }

  public void FaultDump( )
  {
    DataLogManager.log(getSubsystem( ) + ": Temperature is " + powerDistribution.getTemperature( ));
    DataLogManager.log(getSubsystem( ) + ": Input Voltage is " + powerDistribution.getVoltage( ));
    for (int i = 0; i <= 15; i++)
    {
      DataLogManager.log(getSubsystem( ) + ": Chan is " + i + " Current is " + powerDistribution.getCurrent(i));
    }
    DataLogManager.log(getSubsystem( ) + ": Total Current is " + powerDistribution.getTotalCurrent( ));
    DataLogManager.log(getSubsystem( ) + ": Total Power is " + powerDistribution.getTotalPower( ));
    DataLogManager.log(getSubsystem( ) + ": Total Energy is " + powerDistribution.getTotalEnergy( ));

    powerDistribution.resetTotalEnergy( );
    powerDistribution.clearStickyFaults( );
  }

}
