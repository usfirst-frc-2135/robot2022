
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CLConsts;
import frc.robot.subsystems.Climber;

/**
 *
 */
public class ClimberFullClimb extends SequentialCommandGroup
{
  private double m_climbL2Time         = CLConsts.kClimbL2Time;
  private double m_rotateExtendL3Time  = CLConsts.kRotateExtendL3Time;
  private double m_rotateRetractL3Time = CLConsts.kRotateRetractL3Time;
  private double m_climbL3Time         = CLConsts.kClimbL3Time;
  private double m_rotateRetractL4Time = CLConsts.kRotateRetractL4Time;

  public ClimberFullClimb(Climber climber, XboxController gamePad, Button button)
  {
    setName("ClimberFullClimb");

    SmartDashboard.putNumber("CL_climbL2Time", m_climbL2Time);
    SmartDashboard.putNumber("CL_rotateExtendL3Time", m_rotateExtendL3Time);
    SmartDashboard.putNumber("CL_rotateRetractL3Time", m_rotateRetractL3Time);
    SmartDashboard.putNumber("CL_climbL3Time", m_climbL3Time);
    SmartDashboard.putNumber("CL_rotateRetractL4Time", m_rotateRetractL4Time);

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("Climber --- Pull up to Mid rung and engage gate hook ---"),
        new Climber2ClimbToL2(climber),
        new ParallelRaceGroup(
            new WaitCommand(m_climbL2Time), 
            new ClimberTimerOverride(climber, gamePad, button)
        ),

        new PrintCommand("Climber --- Extend and rotate toward High rung ---"),
        new Climber3RotateToL3(climber),
        new ParallelRaceGroup(
            new WaitCommand(m_rotateExtendL3Time), 
            new ClimberTimerOverride(climber, gamePad, button)
        ),

        new PrintCommand("Climber --- Pull gatehook into High rung ---"),
        new Climber5RotateIntoL3(climber),
        new ParallelRaceGroup(
            new WaitCommand(m_rotateRetractL3Time), 
            new ClimberTimerOverride(climber, gamePad, button)
        ),

        new PrintCommand("Climber --- Pull robot up to L3 ---"),
        new Climber6ClimbToL3(climber),
        new ParallelRaceGroup(
            new WaitCommand(m_climbL3Time), 
            new ClimberTimerOverride(climber, gamePad, button)
        ),

        // next rung climb!
        new PrintCommand("Climber --- Extend and rotate toward Traversal rung ---"),
        new Climber3RotateToL3(climber),
        new ParallelRaceGroup(
            new WaitCommand(m_rotateExtendL3Time), 
            new ClimberTimerOverride(climber, gamePad, button)
        ),

        new PrintCommand("Climber --- Pull gatehook into Traversal rung ---"),
        new Climber5RotateIntoL3(climber),
        new ParallelRaceGroup(
            new WaitCommand(m_rotateRetractL4Time), 
            new ClimberTimerOverride(climber, gamePad, button)
        ),

        new PrintCommand("Climber --- Pull robot up to Traversal rung ---"),
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
