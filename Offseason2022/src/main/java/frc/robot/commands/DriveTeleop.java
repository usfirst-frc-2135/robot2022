
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 *
 */
public class DriveTeleop extends CommandBase
{
  private final Drivetrain     m_drivetrain;
  private final XboxController m_driverPad;

  public DriveTeleop(Drivetrain subsystem, XboxController driverPad)
  {
    m_drivetrain = subsystem;
    m_driverPad = driverPad;
    setName("DriveTeleop");
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_drivetrain.driveWithJoysticksInit( );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_drivetrain.driveWithJoysticksExecute(m_driverPad);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_drivetrain.driveWithJoysticksEnd( );
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
