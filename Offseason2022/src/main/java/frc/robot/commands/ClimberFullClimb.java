
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CLConsts;
import frc.robot.subsystems.Climber;

/**
 *
 */
public class ClimberFullClimb extends SequentialCommandGroup
{
  private double m_climbL2Timer         = CLConsts.kClimbL2Timer;
  private double m_rotateExtendL3Timer  = CLConsts.kRotateExtendL3Timer;
  private double m_rotateRetractL3Timer = CLConsts.kRotateRetractL3Timer;
  private double m_climbL3Timer         = CLConsts.kClimbL3Timer;
  private double m_rotateRetractL4Timer = CLConsts.kRotateRetractL4Timer;

  public ClimberFullClimb(Climber climber, XboxController gamePad, Button button)
  {
    setName("ClimberFullClimb");

    SmartDashboard.putNumber("CL_ClimbL2Timer", m_climbL2Timer);
    SmartDashboard.putNumber("CL_RotateExtendL3Timer", m_rotateExtendL3Timer);
    SmartDashboard.putNumber("CL_RotateRetractL3Timer", m_rotateRetractL3Timer);
    SmartDashboard.putNumber("CL_ClimbL3Timer", m_climbL3Timer);
    SmartDashboard.putNumber("CL_RotateRetractL4Timer", m_rotateRetractL4Timer);

    addCommands(
        // Add Commands here:

        // @formatter:off
        new Climber2ClimbToL2(climber),
        new ParallelRaceGroup(
            new WaitCommand(m_climbL2Timer * 1.0), 
            new ClimberTimerOverride(climber, gamePad, button)
        ),
        new Climber3RotateToL3(climber),
        new ParallelRaceGroup(
            new WaitCommand(m_rotateExtendL3Timer * 1.0), 
            new ClimberTimerOverride(climber, gamePad, button)
        ),
        new Climber5RotateIntoL3(climber),
        new ParallelRaceGroup(
            new WaitCommand(m_rotateRetractL3Timer * 1.0), 
            new ClimberTimerOverride(climber, gamePad, button)
        ),
        new Climber6ClimbToL3(climber),
        new ParallelRaceGroup(
            new WaitCommand(m_climbL3Timer * 1.0), 
            new ClimberTimerOverride(climber, gamePad, button)
        ),
        // next rung climb!
        new Climber3RotateToL3(climber),
        new ParallelRaceGroup(
            new WaitCommand(m_rotateExtendL3Timer * 1.0), 
            new ClimberTimerOverride(climber, gamePad, button)
        ),
        new Climber5RotateIntoL3(climber),
        new ParallelRaceGroup(
            new WaitCommand(m_rotateRetractL4Timer * 1.), 
            new ClimberTimerOverride(climber, gamePad, button)
        ),
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
