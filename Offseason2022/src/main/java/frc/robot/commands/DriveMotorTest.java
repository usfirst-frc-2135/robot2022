
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
    // addRequirements(m_subsystem);
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
    RobotContainer robotContainer = RobotContainer.getInstance( );
    if (m_left)
      robotContainer.m_drivetrain.TankDriveVolts(3.0, 0.0);
    else
      robotContainer.m_drivetrain.TankDriveVolts(0.0, 3.0);
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
