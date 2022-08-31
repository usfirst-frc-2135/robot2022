
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/**
 *
 */
public class ClimberSetGatehook extends CommandBase
{
  private boolean m_climberGateHookClosed;

  public ClimberSetGatehook(boolean climberSetGateHook)
  {
    m_climberGateHookClosed = climberSetGateHook;
    setName("ClimberSetGateHook");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    RobotContainer robotContainer = RobotContainer.getInstance( );
    robotContainer.m_climber.setGateHook(m_climberGateHookClosed);
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
