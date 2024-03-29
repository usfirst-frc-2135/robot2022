
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CLConsts.CLHeight;
import frc.robot.subsystems.Climber;

/**
 *
 */
public class ClimberMoveToHeight extends CommandBase
{
  private final Climber m_climber;
  private CLHeight      m_height;

  public ClimberMoveToHeight(Climber climber, CLHeight height)
  {
    m_climber = climber;
    m_height = height;

    setName("ClimberMoveToHeight");
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_climber.moveClimberDistanceInit(m_height);
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
    return false;
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
