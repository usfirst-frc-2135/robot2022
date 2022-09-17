
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VIConsts;
import frc.robot.subsystems.Vision;

/**
 *
 */
public class VisionOnToggle extends CommandBase
{
  private final Vision m_vision;

  public VisionOnToggle(Vision vision)
  {
    m_vision = vision;
    setName("VisionOnToggle");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    if (m_vision.getLEDMode( ) == VIConsts.LED_ON)
    {
      m_vision.setLEDMode(VIConsts.LED_OFF);
      m_vision.setCameraDisplay(VIConsts.PIP_SECONDARY);
    }
    else
    {
      m_vision.setLEDMode(VIConsts.LED_ON);
      m_vision.setCameraDisplay(VIConsts.PIP_MAIN);
    }
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
