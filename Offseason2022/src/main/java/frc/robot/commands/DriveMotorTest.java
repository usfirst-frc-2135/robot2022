
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

/**
 *
 */
public class DriveMotorTest extends CommandBase
{
  private Drivetrain m_drivetrain;
  private boolean    m_left;

  public DriveMotorTest(boolean left, Drivetrain drivetrain)
  {
    m_drivetrain = drivetrain;
    setName("DriveMotorTest");
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    RobotContainer.getInstance( ).m_drivetrain.TankDriveVolts(m_left ? (3.0) : (0.0), m_left ? (0.0) : (3.0));
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
