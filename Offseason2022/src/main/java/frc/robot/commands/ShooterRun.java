
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Shooter.Mode;
import frc.robot.subsystems.Shooter;

/**
 *
 */
public class ShooterRun extends CommandBase
{
  private final Shooter m_shooter;
  private Mode          m_mode;

  public ShooterRun(Mode mode, Shooter subsystem)
  {
    m_shooter = subsystem;
    m_mode = mode;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_shooter.setShooterMode(m_mode);
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
