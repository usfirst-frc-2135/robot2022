
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

/**
 *
 */
public class ClimberTimerOverride extends CommandBase
{
  private Climber        m_climber;
  private XboxController gamePad;

  public ClimberTimerOverride(Climber climber)
  {
    m_climber = climber;
    setName("ClimberTimerOverride");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    gamePad = RobotContainer.getInstance( ).getOperator( );

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
    // return operatorController->GetRawButtonPressed((int)frc::XboxController::Button::kY);

    return gamePad.getRawButtonPressed(XboxController.Button.kY.value);
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
