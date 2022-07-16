
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 *
 */
public class ClimberCalibrate extends CommandBase
{
    private final Climber m_climber;

    public ClimberCalibrate(Climber subsystem)
    {
        m_climber = subsystem;
        addRequirements(m_climber);
    }

    // Called when the command is initially scheduled.
    @Override public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override public void execute() {}

    // Called once the command ends or is interrupted.
    @Override public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override public boolean isFinished()
    {
        return false;
    }

    @Override public boolean runsWhenDisabled()
    {
        return false;
    }
}
