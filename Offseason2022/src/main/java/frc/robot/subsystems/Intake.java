
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.*;

/**
 *
 */
public class Intake extends SubsystemBase {
  
  
    private WPI_TalonFX motorIN8;
  private Solenoid arm;

  
  /**
   *
   */
  public Intake() {
        motorIN8 = new WPI_TalonFX(6);

    arm = new Solenoid(0, PneumaticsModuleType.CTREPCM, 0);
    addChild("Arm", arm);

      }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
}
