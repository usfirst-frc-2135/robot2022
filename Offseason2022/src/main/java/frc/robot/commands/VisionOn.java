
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VIConsts;
import frc.robot.RobotContainer;

/**
 *
 */
public class VisionOn extends CommandBase
{
  private boolean m_mode;
  private boolean m_lightOn;

  public VisionOn(boolean lightOn)
  {
    m_lightOn = lightOn;
    setName("VisionOn");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    // TODO: get method to recognize m_vision
    /*
     * if (m_lightOn) { m_vision.setLEDMode( ); } else { m_vision.setLEDMode(VIConsts.LED_OFF); }
     */
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
