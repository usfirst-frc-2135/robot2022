package frc.robot.team1678.frc2022;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.lib.util.SwerveModuleConstants;
import frc.robot.team1678.frc2022.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConstants;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.util.Units;

public class Constants {

    // toggle constants between comp bot and practice bot ("epsilon")
    public static final boolean isComp = true;
	
	// robot loop time
	public static final double kLooperDt = 0.02;
    
	/* Control Board */
	public static final double kTriggerThreshold = 0.2;

	public static final int kDriveControllerPort = 0;
	public static final int kOperatorControllerPort = 1;

	public static final double stickDeadband = 0.15;

	public static final class SwerveConstants {
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.75);
        public static final double wheelBase = Units.inchesToMeters(20.75);

        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 6.75;
        public static final double angleGearRatio = 21.43;
        
        public static final edu.wpi.first.math.geometry.Translation2d[] swerveModuleLocations = {
            new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        };

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(swerveModuleLocations);

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.3;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.32 / 12);
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; // meters per second
        public static final double maxAngularVelocity = 10.0;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Controller Invert */
        public static final boolean invertYAxis = false;
        public static final boolean invertRAxis = false;
        public static final boolean invertXAxis = false; 


        /*** MODULE SPECIFIC CONSTANTS ***/

        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final double epsilonAngleOffset = 239.06;
            public static final double compAngleOffset = 68.37; 

            public static SwerveModuleConstants SwerveModuleConstants() {
                return new SwerveModuleConstants(Ports.FL_DRIVE, Ports.FL_ROTATION, Ports.FL_CANCODER,
                        isComp ? compAngleOffset : epsilonAngleOffset);
            }
        }
        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final double epsilonAngleOffset = 339.96;
            public static final double compAngleOffset = 312.53; 
            
            public static SwerveModuleConstants SwerveModuleConstants() {
                return new SwerveModuleConstants(Ports.FR_DRIVE, Ports.FR_ROTATION, Ports.FR_CANCODER,
                        isComp ? compAngleOffset : epsilonAngleOffset);
            }
        }
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final double epsilonAngleOffset = 317.20;
            public static final double compAngleOffset = 357.97;

            public static SwerveModuleConstants SwerveModuleConstants() {
                return new SwerveModuleConstants(Ports.BL_DRIVE, Ports.BL_ROTATION, Ports.BL_CANCODER,
                        isComp ? compAngleOffset : epsilonAngleOffset);
            }
        }
        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final double epsilonAngleOffset = 311.22;
            public static final double compAngleOffset = 295.2;

            public static SwerveModuleConstants SwerveModuleConstants() {
                return new SwerveModuleConstants(Ports.BR_DRIVE, Ports.BR_ROTATION, Ports.BR_CANCODER,
                        isComp ? compAngleOffset : epsilonAngleOffset);
            }
        }
    }
	
	public static final class SnapConstants {
        public static final double kP = 5.0; 
        public static final double kI = 0;
        public static final double kD = 0.0;
        public static final double kTimeout = 0.25;
        public static final double kEpsilon = 1.0;

        // Constraints for the profiled angle controller
        public static final double kMaxAngularSpeedRadiansPerSecond = 2.0 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class VisionAlignConstants {
        public static final double kP = 6.37;
        public static final double kI = 0.0;
        public static final double kD = 0.10;
        public static final double kTimeout = 0.25;
        public static final double kEpsilon = 5.0;

        // Constraints for the profiled angle controller
        public static final double kMaxAngularSpeedRadiansPerSecond = 2.0 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 10.0 * Math.PI;
        
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class AutoConstants {
        public static final double kSlowSpeedMetersPerSecond = 1.7;
        public static final double kSlowAccelerationMetersPerSecondSquared = 2.0;

        public static final double kMaxSpeedMetersPerSecond = 2.2; 
        public static final double kMaxAccelerationMetersPerSecondSquared = 2.3;
        
        public static final double kSlowMaxAngularSpeedRadiansPerSecond = 0.8 * Math.PI;
        public static final double kSlowMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kSlowMaxAngularSpeedRadiansPerSecond, 2);

        public static final double kMaxAngularSpeedRadiansPerSecond = 1.2 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 5;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kSlowThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kSlowMaxAngularSpeedRadiansPerSecond, kSlowMaxAngularSpeedRadiansPerSecondSquared);


        public static TrajectoryConfig createConfig(double maxSpeed, double maxAccel, double startSpeed, double endSpeed) {
            TrajectoryConfig config = new TrajectoryConfig(maxSpeed, maxAccel);
            config.setKinematics(Constants.SwerveConstants.swerveKinematics);
            config.setStartVelocity(startSpeed);
            config.setEndVelocity(endSpeed);
            config.addConstraint(new CentripetalAccelerationConstraint(3.0));
            return config;
        }

        // Trajectory Speed Configs
        public static final TrajectoryConfig defaultSpeedConfig =
                new TrajectoryConfig(
                        kMaxSpeedMetersPerSecond,
                        kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(Constants.SwerveConstants.swerveKinematics);

        public static final TrajectoryConfig slowSpeedConfig =
                new TrajectoryConfig(
                kSlowSpeedMetersPerSecond,
                kSlowAccelerationMetersPerSecondSquared)    
                .setKinematics(Constants.SwerveConstants.swerveKinematics)
                        .setStartVelocity(0)
                        .setEndVelocity(0); 
    }

    public static final class VisionConstants {

		public static final double kHorizontalFOV = 59.6; // degrees
		public static final double kVerticalFOV = 49.7; // degrees
		public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds
        
        // lookahead time
        public static final double kLookaheadTime = 0.0; // 1.10 as latest

        /* Goal Tracker Constants */
        public static final double kMaxTrackerDistance = 8.0;
        public static final double kMaxGoalTrackAge = 10.0;
        public static final double kMaxGoalTrackSmoothingTime = 1.5;
        public static final double kCameraFrameRate = 90.0;

        public static final double kTrackStabilityWeight = 0.0;
        public static final double kTrackAgeWeight = 10.0;
        public static final double kTrackSwitchingWeight = 100.0;

        public static final int kDefaultPipeline = 0;
        public static final double kGoalHeight = 2.63; // meters
        public static final double kGoalRadius = 0.678; // meters
	}

    /*** SUBSYSTEM CONSTANTS ***/

    // Timeout constants
    public static final int kLongCANTimeoutMs = 100;
    public static final int kCANTimeoutMs = 10;
}
