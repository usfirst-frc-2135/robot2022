
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CLConsts;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TowerConveyor;

/**
 *
 */
public class ClimberL3ToL4 extends SequentialCommandGroup
{
  private double m_rotateExtendL3Timer  = CLConsts.kRotateExtendL3Timer;
  private double m_rotateRetractL4Timer = CLConsts.kRotateRetractL4Timer;

  public ClimberL3ToL4(Climber climber, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter)
  {
    setName("ClimberL3ToL4");

    addCommands(
        // Add Commands here:

        // @formatter:off
      new Climber3RotateToL3(climber),
      new WaitCommand(m_rotateExtendL3Timer),
      new Climber5RotateIntoL3(climber),
      new WaitCommand(m_rotateRetractL4Timer),
      new Climber7ClimbToL4(climber)
      // @formatter:on

    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
