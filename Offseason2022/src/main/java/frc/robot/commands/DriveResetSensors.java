
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 *
 */
public class DriveResetSensors extends CommandBase
{
  private Drivetrain m_drivetrain;

  public DriveResetSensors(Drivetrain drivetrain)
  {

    drivetrain = m_drivetrain;
    setName("ResetDriveSensors");

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_drivetrain.resetSensors( );
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
