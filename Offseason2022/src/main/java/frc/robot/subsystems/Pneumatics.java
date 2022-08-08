
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 *
 */
public class Pneumatics extends SubsystemBase
{

  // TODO: should compressor be defined locally or in constructor(it is not used)?
  private Compressor              compressor       = new Compressor(PneumaticsModuleType.CTREPCM);

  private PneumaticsControlModule pcm              = new PneumaticsControlModule(0);

  private final int               pneumaticsDebug  = 0;
  private int                     periodicInterval = 0;

  /**
   *
   */
  public Pneumatics( )
  {
    setName("Pneumatics");
    setSubsystem("Pneumatics");

    // compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    addChild("Compressor", compressor);
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    if ((pneumaticsDebug > 0) && (periodicInterval++ % 5 == 0))
    {
      SmartDashboard.putNumber("PCM_Output_Comp", pcm.getCompressorCurrent( ));
    }

  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run when in simulation
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public void FaultDump( )
  {
    // Print out PCM faults and clear sticky ones
    DataLogManager.log(getSubsystem( ) + ": ----- PCM FAULTS --------------");

    if (pcm.getCompressorCurrentTooHighFault( ))
      DataLogManager.log(getSubsystem( ) + ": Warn - CurrentTooHighFault");
    if (pcm.getCompressorNotConnectedFault( ))
      DataLogManager.log(getSubsystem( ) + ": Warn - CompressorNotConnectedFault");
    if (pcm.getCompressorShortedFault( ))
      DataLogManager.log(getSubsystem( ) + ": Warn - CompressorShortedFault");
    if (pcm.getSolenoidVoltageFault( ))
      DataLogManager.log(getSubsystem( ) + ": Warn - SolenoidVoltageFault");

    pcm.clearAllStickyFaults( );
  }

  public void initialize( )
  {
    DataLogManager.log(getSubsystem( ) + ": subsystem initialized!");
  }
}
