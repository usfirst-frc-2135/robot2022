
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/**
 *
 */
public class DriveQuickturn extends CommandBase
{
  public DriveQuickturn( )
  {
    setName("DriveQuickturn");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    RobotContainer.getInstance( ).m_drivetrain.moveSetQuickTurn(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    RobotContainer.getInstance( ).m_drivetrain.moveSetQuickTurn(false);
  }

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
