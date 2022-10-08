
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DTConsts;

/**
 *
 */
public class Drivetrain extends SubsystemBase
{
  private WPI_TalonFX                 driveLF1;
  private WPI_TalonFX                 steerLF2;
  private WPI_TalonFX                 driveLR3;
  private WPI_TalonFX                 steerLR4;
  private WPI_TalonFX                 driveRF5;
  private WPI_TalonFX                 steerRF6;
  private WPI_TalonFX                 driveRR7;
  private WPI_TalonFX                 steerRR8;
  private Pigeon2                     pigeonIMU;

  private final XboxController        m_controller         = new XboxController(0);
  private final Drivetrain            m_swerve             = new Drivetrain( );

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter       m_xspeedLimiter      = new SlewRateLimiter(3);
  private final SlewRateLimiter       m_yspeedLimiter      = new SlewRateLimiter(3);
  private final SlewRateLimiter       m_rotLimiter         = new SlewRateLimiter(3);

  public static final double          kMaxSpeed            = 3.0;             // 3 meters per second
  public static final double          kMaxAngularSpeed     = Math.PI;         // 1/2 rotation per second

  private final Translation2d         m_frontLeftLocation  = new Translation2d(0.381, 0.381);
  private final Translation2d         m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d         m_backLeftLocation   = new Translation2d(-0.381, 0.381);
  private final Translation2d         m_backRightLocation  = new Translation2d(-0.381, -0.381);

  private final SwerveModule          m_frontLeft          =
      new SwerveModule(DTConsts.kLFDrive1CANID, DTConsts.kLFSteer2CANID, 0, 1, 2, 3);
  private final SwerveModule          m_frontRight         =
      new SwerveModule(DTConsts.kRFDrive3CANID, DTConsts.kRFSteer4CANID, 4, 5, 6, 7);
  private final SwerveModule          m_backLeft           =
      new SwerveModule(DTConsts.kLRDrive5CANID, DTConsts.kLRSteer6CANID, 8, 9, 10, 11);
  private final SwerveModule          m_backRight          =
      new SwerveModule(DTConsts.kRRDrive7CANID, DTConsts.kRRSteer8CANID, 12, 13, 14, 15);

  private final AnalogGyro            m_gyro               = new AnalogGyro(0);

  private final SwerveDriveKinematics m_kinematics         =
      new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry   m_odometry           = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d( ));

  /**
  *
  */
  public Drivetrain( )
  {
    driveLF1 = new WPI_TalonFX(1);
    steerLF2 = new WPI_TalonFX(2);
    driveLR3 = new WPI_TalonFX(3);
    steerLR4 = new WPI_TalonFX(4);

    driveRF5 = new WPI_TalonFX(5);
    steerRF6 = new WPI_TalonFX(6);
    driveRR7 = new WPI_TalonFX(7);
    steerRR8 = new WPI_TalonFX(8);

    pigeonIMU = new Pigeon2(0);

    m_gyro.reset( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
    m_swerve.updateOdometry( );
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run when in simulation
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed
   *          Speed of the robot in the x direction (forward).
   * @param ySpeed
   *          Speed of the robot in the y direction (sideways).
   * @param rot
   *          Angular rate of the robot.
   * @param fieldRelative
   *          Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative)
  {
    var swerveModuleStates = m_kinematics
        .toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d( ))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry( )
  {
    m_odometry.update(m_gyro.getRotation2d( ), m_frontLeft.getState( ), m_frontRight.getState( ), m_backLeft.getState( ),
        m_backRight.getState( ));
  }

  public void driveWithJoystick(XboxController driverPad, boolean fieldRelative)
  {
    // Get x speed. Invert this because Xbox controllers return negative values when pushing forward.
    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(driverPad.getLeftY( ), 0.02)) * Drivetrain.kMaxSpeed;

    // Get y speed or sideways/strafe speed. Invert this because a positive value is needed when
    // pulling left. Xbox controllers return positive values when pulling right by default.
    final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(driverPad.getLeftX( ), 0.02)) * Drivetrain.kMaxSpeed;

    // Get rate of angular rotation. Invert this because a positive value is needed when pulling to
    // the left (CCW is positive in mathematics). Xbox controllers return positive values when pulling
    // to the right by default.
    final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(driverPad.getRightX( ), 0.02)) * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  public double getDistanceMeters(WPI_TalonFX driveMotor)
  {
    return nativeUnitsToMeters(driveMotor.getSelectedSensorPosition( ));
  }

  private double getVelocityMPS(WPI_TalonFX driveMotor)
  {
    return nativeUnitsToMPS(driveMotor.getSelectedSensorVelocity( ));
  }

  private double nativeUnitsToMeters(double nativeUnits)
  {
    return nativeUnits * DTConsts.kEncoderMetersPerCount;
  }

  private double nativeUnitsToMPS(double nativeUnitsVelocity)
  {
    return nativeUnitsVelocity * DTConsts.kEncoderMetersPerCount * 10;
  }
}
