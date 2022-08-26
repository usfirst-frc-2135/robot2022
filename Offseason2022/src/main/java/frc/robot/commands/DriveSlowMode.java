
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/**
 *
 */
public class DriveSlowMode extends CommandBase
{
  private boolean m_driveSlow;

  public DriveSlowMode(boolean driveSlow)
  {
    m_driveSlow = driveSlow;
    setName("DriveSlowMode");
    // addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    RobotContainer robotContainer = RobotContainer.getInstance( );
    robotContainer.m_drivetrain.setDriveSlowMode(m_driveSlow);
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
