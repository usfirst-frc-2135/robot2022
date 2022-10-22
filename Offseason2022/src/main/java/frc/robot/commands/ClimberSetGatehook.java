
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 *
 */
public class ClimberSetGatehook extends CommandBase
{
  private final Climber m_climber;
  private boolean       m_closeHook;
  private Timer         m_timer;

  public ClimberSetGatehook(Climber climber, boolean closeHook)
  {
    m_climber = climber;
    m_closeHook = closeHook;

    setName("ClimberSetGateHook");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_climber.setGateHook(m_closeHook, m_timer);
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
    return false;
  }
}
