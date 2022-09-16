
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
  private double m_climbL2Time         = CLConsts.kClimbL2Time;
  private double m_rotateExtendL3Time  = CLConsts.kRotateExtendL3Time;
  private double m_rotateRetractL3Time = CLConsts.kRotateRetractL3Time;
  private double m_climbL3Time         = CLConsts.kClimbL3Time;

  public ClimberL2ToL3(Climber climber, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter)
  {
    setName("ClimberL2ToL3");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new Climber2ClimbToL2(climber),
        new WaitCommand(m_climbL2Time), // check to see if timer is necessary here
        new Climber3RotateToL3(climber),
        new WaitCommand(m_rotateExtendL3Time),
        new Climber5RotateIntoL3(climber),
        new WaitCommand(m_rotateRetractL3Time),
        new Climber6ClimbToL3(climber),
        new WaitCommand(m_climbL3Time) 
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
