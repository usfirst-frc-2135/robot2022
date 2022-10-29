
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 *
 */
public class AutoDrivePath extends CommandBase
{
  private final Drivetrain m_drivetrain;
  private final boolean    m_useInitialPose;

  private String           m_trajectoryJSON;
  private Trajectory       m_trajectory;

  public AutoDrivePath(Drivetrain drivetrain, String pathName, boolean useInitialPose)
  {
    m_drivetrain = drivetrain;
    m_useInitialPose = useInitialPose;

    setName("AutoDrivePath");
    addRequirements(m_drivetrain);

    // Get our trajectory
    m_trajectoryJSON = Filesystem.getDeployDirectory( ).getAbsolutePath( ).toString( ) + "/output/" + pathName + ".wpilib.json";

    try
    {
      DataLogManager.log(String.format("%s: TrajPath %s", getName( ), m_trajectoryJSON));
      Path trajectoryPath = Filesystem.getDeployDirectory( ).toPath( ).resolve(m_trajectoryJSON);
      m_trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      DataLogManager.log(String.format("%s: Num states %2d Total time %.3f secs", getName( ), m_trajectory.getStates( ).size( ),
          m_trajectory.getTotalTimeSeconds( )));
    }
    catch (IOException ex)
    {
      DataLogManager.log(String.format("%s: Unable to open %s", getName( ), m_trajectoryJSON));
      DriverStation.reportError("Unable to open trajectory: " + m_trajectoryJSON, ex.getStackTrace( ));
    }

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Running", getName( ), m_trajectoryJSON));
    m_drivetrain.driveWithPathFollowerInit(m_trajectory, m_useInitialPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_drivetrain.driveWithPathFollowerExecute( );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_drivetrain.driveWithPathFollowerEnd( );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return m_drivetrain.driveWithPathFollowerIsFinished( );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
