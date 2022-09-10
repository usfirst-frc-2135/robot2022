
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.VIConsts;
import frc.robot.subsystems.Vision;

/**
 *
 */
public class VisionOn extends CommandBase
{

  private final boolean m_lightOn;

  public VisionOn(boolean lightOn)
  {
    m_lightOn = lightOn;
    setName("VisionOn");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    RobotContainer.getInstance( ).m_vision.setLEDMode((m_lightOn) ? VIConsts.LED_ON : VIConsts.LED_OFF);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return true;
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return true;
  }
}
