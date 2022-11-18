
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

/**
 *
 */
public class DriveTeleop extends CommandBase
{
  private final Swerve         m_swerve;
  private final XboxController m_driverPad;
  private final int            m_speedAxis;
  private final int            m_rotationAxis;

  public DriveTeleop(Swerve swerve, XboxController driverPad, int speedAxis, int rotationAxis)
  {
    m_swerve = swerve;
    m_driverPad = driverPad;
    m_speedAxis = speedAxis;
    m_rotationAxis = rotationAxis;

    setName("DriveTeleop");
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_swerve.driveWithJoystick(m_driverPad, true);
  }

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
