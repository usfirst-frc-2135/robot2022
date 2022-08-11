
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake.Mode;
import frc.robot.frc2135.RobotConfig;

/**
 *
 */
public class Intake extends SubsystemBase
{
  private WPI_TalonFX motorIN8      = new WPI_TalonFX(8);
  private Solenoid    arm           = new Solenoid(0, PneumaticsModuleType.CTREPCM, 0);
  private boolean     talonValidIN8 = false;
  private double      acquireSpeed;
  private double      expelSpeed;

  /**
   *
   */
  public Intake( )
  {
    setName("Intake");
    setSubsystem("Intake");
    addChild("Arm", arm);

    SmartDashboard.putBoolean("HL_IN8Valid", talonValidIN8);

    // Check if solenoids are functional or blacklisted
    if (arm.isDisabled( ))
      DataLogManager.log(getSubsystem( ) + "Deploy Solenoid is BLACKLISTED");
    else
      DataLogManager.log(getSubsystem( ) + "Deploy Solenoid is FUNCTIONAL");

    RobotConfig config = RobotConfig.getInstance( );
    acquireSpeed = config.getValueAsDouble("IN_AcquireSpeed", 0.6);
    expelSpeed = config.getValueAsDouble("IN_ExpelSpeed", -0.6);

    motorIN8.setInverted(true);
    motorIN8.setSafetyEnabled(false);

    motorIN8.set(0.0);

    initialize( );

  }

  public void initialize( )
  {
    DataLogManager.log(getSubsystem( ) + ": subsystem initialized!");
    setIntakeSpeed(Mode.INTAKE_STOP);
    setDeployerSolenoid(false);
  }

  public void setIntakeSpeed(Mode mode)
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
        output = acquireSpeed;
        break;
      case INTAKE_EXPEL :
        strName = "EXPEL";
        output = expelSpeed;
        break;
    }

    // Set speed of intake and the percent output
    DataLogManager.log(getSubsystem( ) + ": IN Set Speed - {}" + strName);

    motorIN8.set(output);
  }

  public void setDeployerSolenoid(boolean extended)
  {
    if (extended = true)
    {
      DataLogManager.log(getSubsystem( ) + "IN Intake {} DEPLOY");
    }
    else
    {
      DataLogManager.log(getSubsystem( ) + "IN Intake {} STOW");
    }
    // DataLogManager.log("IN Intake {}", (extended) ? "DEPLOY" : "STOW");
    SmartDashboard.putBoolean("IN_Deployed", extended);

    arm.set(extended);
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
