
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
public class ClimberL2ToL3 extends SequentialCommandGroup
{
  private double m_climbL2Timer         = CLConsts.kClimbL2Timer;
  private double m_rotateExtendL3Timer  = CLConsts.kRotateExtendL3Timer;
  private double m_rotateRetractL3Timer = CLConsts.kRotateRetractL3Timer;
  private double m_climbL3Timer         = CLConsts.kClimbL3Timer;

  public ClimberL2ToL3(Climber climber, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter)
  {
    setName("ClimberL2ToL3");

    addCommands(
        // Add Commands here:

        // @formatter:off
      new Climber2ClimbToL2(climber),
      new WaitCommand(m_climbL2Timer), // check to see if timer is necessary here
      new Climber3RotateToL3(climber),
      new WaitCommand(m_rotateExtendL3Timer),
      new Climber5RotateIntoL3(climber),
      new WaitCommand(m_rotateRetractL3Timer),
      new Climber6ClimbToL3(climber),
      new WaitCommand(m_climbL3Timer)
      
      // @formatter:on

    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
