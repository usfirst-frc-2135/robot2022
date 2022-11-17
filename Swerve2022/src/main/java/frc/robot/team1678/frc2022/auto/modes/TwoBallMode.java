package frc.robot.team1678.frc2022.auto.modes;

import frc.robot.team1678.frc2022.Constants;
import frc.robot.team1678.frc2022.auto.AutoModeEndedException;
import frc.robot.team1678.frc2022.auto.AutoTrajectoryReader;
import frc.robot.team1678.frc2022.auto.actions.LambdaAction;
import frc.robot.team1678.frc2022.auto.actions.SwerveTrajectoryAction;
import frc.robot.team1678.frc2022.auto.actions.VisionAlignAction;
import frc.robot.team1678.frc2022.auto.actions.WaitAction;
import frc.robot.team1678.frc2022.subsystems.Superstructure;
import frc.robot.team1678.frc2022.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TwoBallMode extends AutoModeBase {

    // Swerve instance
    private final Swerve mSwerve = Swerve.getInstance();
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    // trajectory
    private Trajectory traj_path;

    // required PathWeaver file path
    String file_path = "paths/TwoBallPaths/2 by 2 A.path";

    // trajectory action
    SwerveTrajectoryAction driveToShotPose;

    public TwoBallMode() {

        SmartDashboard.putBoolean("Auto Finished", false);

        // define theta controller for robot heading
        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                                                        Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // read trajectories from PathWeaver and generate trajectory actions
        traj_path = AutoTrajectoryReader.generateTrajectoryFromFile(file_path, Constants.AutoConstants.defaultSpeedConfig);
        driveToShotPose = new SwerveTrajectoryAction(traj_path,
                                                           mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                           new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                           new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                           thetaController,
                                                           () -> Rotation2d.fromDegrees(135.0),
                                                           mSwerve::getWantAutoVisionAim,
                                                           mSwerve::setModuleStates);

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running two ball mode auto!");
        SmartDashboard.putBoolean("Auto Finished", false);

        // start spinning up for shot
        runAction(new LambdaAction(() -> mSuperstructure.setWantPrep(true)));

        // reset odometry at the start of the trajectory
        runAction(new LambdaAction(() -> mSwerve.resetOdometry(new Pose2d(driveToShotPose.getInitialPose().getX(),
                                                                          driveToShotPose.getInitialPose().getY(),
                                                                          Rotation2d.fromDegrees(135.0)))));
 
        // start intaking
        runAction(new LambdaAction(() -> mSuperstructure.setWantIntake(true)));
         
        // drive to intake second cargo
        runAction(driveToShotPose);

        // start vision aiming to align drivetrain to target
        runAction(new VisionAlignAction(Constants.SwerveConstants.swerveKinematics));
 
        // wait for 0.5 seconds before shooting
        runAction(new WaitAction(1.0));

        // shoot two cargo
        runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(true)));
        runAction(new WaitAction(3.0));
        runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(false)));

        // ready for teleop
        runAction(new LambdaAction(() -> mSuperstructure.setInitialTeleopStates()));

        System.out.println("Finished auto!");
        SmartDashboard.putBoolean("Auto Finished", true);
    }

    @Override
    public Pose2d getStartingPose() {
        return driveToShotPose.getInitialPose();
    }
}
