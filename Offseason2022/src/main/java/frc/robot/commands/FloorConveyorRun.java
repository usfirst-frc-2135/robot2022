
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FloorConveyor;

/**
 *
 */
public class FloorConveyorRun extends CommandBase
{
  private final FloorConveyor m_floorConveyor;
  private int                 m_direction;

  public FloorConveyorRun(int direction, FloorConveyor subsystem)
  {
    m_floorConveyor = subsystem;
    m_direction = direction;
    addRequirements(m_floorConveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {}

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
