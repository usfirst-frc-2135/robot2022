
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TowerConveyor;

/**
 *
 */
public class Climber0Stow extends SequentialCommandGroup
{
  private final Climber       m_climber;
  private final Intake        m_intake;
  private final FloorConveyor m_fConv;
  private final TowerConveyor m_tConv;
  private final Shooter       m_shooter;

  public Climber0Stow(Climber climber, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter)
  {
    m_climber = climber;
    m_intake = intake;
    m_fConv = fConv;
    m_tConv = tConv;
    m_shooter = shooter;

    addCommands(
    // Add Commands here:
    // Also add parallel commands using the
    //
    // addCommands(
    // new command1(argsN, subsystem),
    // parallel(
    // new command2(argsN, subsystem),
    // new command3(argsN, subsystem)
    // )
    // );

    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
