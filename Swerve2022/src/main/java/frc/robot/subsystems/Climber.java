
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.*;

/**
 *
 */
public class Climber extends SubsystemBase
{
  private WPI_TalonFX  motorCL18;
  private WPI_TalonFX  motorCL19;
  private Solenoid     gateHook;
  private CANCoder     gateHookAngle;
  private DigitalInput downLimitLeft;
  private DigitalInput downLimitRight;

  /**
  *
  */
  public Climber( )
  {
    motorCL18 = new WPI_TalonFX(18);

    motorCL19 = new WPI_TalonFX(19);

    gateHook = new Solenoid(0, PneumaticsModuleType.CTREPCM, 1);
    addChild("GateHook", gateHook);

    gateHookAngle = new CANCoder(9);

    downLimitLeft = new DigitalInput(1);
    addChild("DownLimitLeft", downLimitLeft);

    downLimitRight = new DigitalInput(2);
    addChild("DownLimitRight", downLimitRight);
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
}
