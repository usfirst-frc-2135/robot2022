
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VIConsts;
import frc.robot.RobotContainer;

/**
 *
 */
public class VisionOnToggle extends CommandBase
{
  public VisionOnToggle( )
  {
    setName("VisionOnToggle");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    RobotContainer robotContainer = RobotContainer.getInstance( );

    if (robotContainer.m_vision.getLEDMode( ) == VIConsts.LED_ON)
    {
      robotContainer.m_vision.setLEDMode(VIConsts.LED_OFF);
      robotContainer.m_vision.setCameraDisplay(VIConsts.PIP_SECONDARY);
    }
    else
    {
      robotContainer.m_vision.setLEDMode(VIConsts.LED_ON);
      robotContainer.m_vision.setCameraDisplay(VIConsts.PIP_MAIN);
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
