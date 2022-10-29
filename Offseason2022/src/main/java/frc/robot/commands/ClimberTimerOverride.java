
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 *
 */
public class ClimberTimerOverride extends CommandBase
{
  private XboxController m_gamePad;
  private Button         m_button;

  public ClimberTimerOverride(Climber climber, XboxController gamePad, Button button)
  {
    m_gamePad = gamePad;
    m_button = button;

    setName("ClimberTimerOverride");
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
    return m_gamePad.getRawButtonPressed(m_button.value);
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
