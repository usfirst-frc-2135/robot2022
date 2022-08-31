
package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants
{
  public static final class Falcon500
  {
    public static int          kMaxRPM               = 6380;             // free speed for Falcon 500 motor
    public static final double kEncoderCPR           = 2048;             // CPR is 2048 from Falcon 500 Manual
    public static final int    kTalonReqVersion      = ((22 * 256) + 0); // Talon version is 22.0
    public static final int    kPigeonReqVersion     = ((22 * 256) + 0); // Pigeon IMU version is 22.0

    // Input current limit settings
    public static final double kSupplyCurrentLimit   = 45.0;  // Default supply current limit (after trigger)
    public static final double kSupplyTriggerCurrent = 45.0;  // Trigger current that will cause limiting
    public static final double kSupplyTriggerTime    = 0.001; // Time duration of trigger that will causing limiting

    // Output current limit settings
    public static final double kStatorCurrentLimit   = 80.0;  // Default supply current limit (after trigger)
    public static final double kStatorTriggerCurrent = 80.0;  // Default trigger current that will cause limiting
    public static final double kStatorTriggerTime    = 0.001; // Default time duration of trigger that will causing limiting
  }

  public static final class DTConsts
  {
    public static final int    kL1CANID               = 1;
    public static final int    kL2CANID               = 2;
    public static final int    kR3CANID               = 3;
    public static final int    kR4CANID               = 4;

    // Kinematics values for 2135 Bebula - 2019 B-bo
    public static final double ks                     = 0.65; // units
    public static final double kv                     = 2.84; // units
    public static final double ka                     = 0.309;
    public static final double KvAngular              = 1.5;
    public static final double KaAngular              = 0.3;

    public static final double kWheelDiaMeters        = 4.0;  // Units library does the conversion
    public static final double kGearRatio             = 8.45; // Gear reduction
    public static final double kTrackWidthMeters      = 0.6477; // Measured track width

    public static final double kEncoderMetersPerCount = (kWheelDiaMeters * Math.PI) / (Falcon500.kEncoderCPR) / kGearRatio;
  }

  public static final class INConsts
  {
    // public static final int kIN8CANID = 6;
    public static final int    kINPWM1         = 1;
    public static final int    kArmSolenoid    = 0;

    public static final double kINAcquireSpeed = 0.6;
    public static final double kINExpelSpeed   = -0.6;

    public enum INMode
    {
      INTAKE_STOP,    // Stop intake motor
      INTAKE_ACQUIRE, // Acquire game pieces
      INTAKE_EXPEL,   // Expel game pieces
    }
  }

  public static final class FCConsts
  {
    public static final int    kFC8CANID           = 8;

    public static final double kFCAcquireSpeed     = 1.0;
    public static final double kFCAcquireSpeedSlow = 0.2;
    public static final double kFCExpelSpeedFast   = -1.0;

    public enum FCMode
    {
      FCONVEYOR_STOP,       // Stop floor conveyor motor
      FCONVEYOR_ACQUIRE,    // Aquire game pieces
      FCONVEYOR_EXPEL,      // Expel game pieces
      FCONVEYOR_EXPEL_FAST, // Expel Fast
    }
  }

  public static final class TCConsts
  {
    public static final int    kTC9CANID           = 9;
    public static final int    kCargoDIO           = 2;

    public static final double kTCAcquireSpeed     = 1.0;
    public static final double kTCAcquireSpeedSlow = 0.2;
    public static final double kTCExpelSpeed       = -0.2;
    public static final double kTCExpelSpeedFast   = -1.0;

    public enum TCMode
    {
      TCONVEYOR_STOP,         // Conveyor stop
      TCONVEYOR_ACQUIRE,      // Conveyor moves game pieces to shooter
      TCONVEYOR_ACQUIRE_SLOW, // Conveyor moves during game piece intake
      TCONVEYOR_EXPEL,        // Conveyor moves game pieces to hopper
      TCONVEYOR_EXPEL_FAST,   // Conveyor moves game pieces to hopper
    }
  }

  public static final class SHConsts
  {
    public static final int    kSH11CANID               = 11;

    public static final double kFlywheelGearRatio       = (18.0 / 12.0);
    public static final double kFlywheelCPR             = Falcon500.kEncoderCPR * kFlywheelGearRatio;

    public static final double kFlywheelPidKf           = 0.0475;
    public static final double kFlywheelPidKp           = 0.05;
    public static final double kFlywheelPidKi           = 0.0;
    public static final double kFlywheelPidKd           = 0.0;
    public static final double kFlywheelNeutralDeadband = 0.004;

    public static final double kFlywheelToleranceRPM    = 200.0;     // Tolerance band around target RPM
    public static final double kFlywheelPrimeRPM        = 1000.0;    // RPM for priming the shooter
    public static final double kFlywheelLowerTargetRPM  = 1450.0;    // RPM for lower hub
    public static final double kFlywheelUpperTargetRPM  = 3000.0;    // RPM for upper hub

    public static final double kReverseRPMThreshold     = 20.0;      // RPM threshold for allowing reverse of motor
    public static final double kFlywheelReverseRPM      = -1000.0;   // RPM for reversing out game pieces

    public enum Mode
    {
      SHOOTER_REVERSE,    // Shooter runs in reverse direction to handle jams
      SHOOTER_STOP,       // Shooter is stopped
      SHOOTER_PRIME,      // Shooter ramped to an initial speed before shooting
      SHOOTER_LOWERHUB,   // Shooter at speed for low hub
      SHOOTER_UPPERHUB,   // Shooter at speed for high hub
    }
  }

  public static final class CLConsts
  {
    public static final int    kCL14LeftCANID       = 14;
    public static final int    kCL15RightCANID      = 15;
    public static final int    kCLLeftLimitDIO      = 0;
    public static final int    kCLRightLimitDIO     = 1;
    public static final int    kGateHookSolenod     = 1;
    public static final int    kCLCancoderID        = 0;

    // Config file parameters
    public static final int    kMMVelocity          = 21776;  // Climber motion magic velocity
    public static final int    kMMAcceleration      = 43552;  // Climber motion magic acceleration
    public static final int    kMMSCurveStrength    = 0;      // Climber motion magic S curve smoothing strength
    public static final double kCLPidKf             = 0.0496; // Climber PID force constant
    public static final double kCLPidKp             = 0.500;  // Climber PID proportional constant
    public static final double kCLPidKi             = 0.0;    // Climber PID integral constant
    public static final double kCLPidKd             = 0.0;    // Climber PID derivative constant

    public static final double kStowHeight          = 0.10;   // 0.25 inches
    public static final double kExtendL2            = 29.0;   // 29 inches
    public static final double kRotateL3            = 31.25;  // 21 inches
    public static final double kRaiseL4             = 15.0;   // 25.25 inches
    public static final double kGatehookRestHeight  = 4.0;    // 0.35 inches

    public static final int    kCLAllowedError      = 0;      // Climber PID allowable closed loop error in counts
    public static final double kCLToleranceInches   = 0.25;   // Climber PID tolerance in inches
    public static final double kClimberMaxHeight    = 36.0;   // Climber maximum allowable height
    public static final double kClimberMinHeight    = 0.0;    // Climber minimum allowable height

    public static final double kClimberRolloutRatio = 0.432;  // inches per shaft rotation
    public static final double kInchesPerCount      = kClimberRolloutRatio * (1.0 / (double) Falcon500.kEncoderCPR);

    public static final double kClimberGearRatio    = 10.0;   // 1/rollout in meters
    public static final double kClimberCPR          = Falcon500.kEncoderCPR / kClimberGearRatio;

    public enum Height
    {                       // Climber subsystem movement states
      NOCHANGE_HEIGHT,      // No change in climber height--maintain current position
      STOW_HEIGHT,          // Move to stow height
      EXTEND_L2_HEIGHT,     // Move to extend to L2 height
      ROTATE_L3_HEIGHT,     // Move to rotate to L3 height
      GATEHOOK_REST_HEIGHT, // Move to lower on L3 height so gate hooks clamp
      RAISE_L4_HEIGHT       // Move to extend on last rung ~6 inches
    }

    public enum CLMode
    {
      CLIMBER_INIT,         // Initialize climber
      CLIMBER_DOWN,         // Move climber down
      CLIMBER_STOPPED,      // Stop and hold position
      CLIMBER_UP            // Move climber up
    }
  }

  public static final class VIConsts
  {
    // Limelight-defined streaming states
    public static final int    STANDARD       = 0;  // Both cameras side-by-side
    public static final int    PIP_MAIN       = 1;  // Limelight with second camera inset
    public static final int    PIP_SECONDARY  = 2;  // Second camera with limelight inset

    // Limelight-defined LED mode states
    public static final int    LED_CUR_MODE   = 0;
    public static final int    LED_OFF        = 1;
    public static final int    LED_BLINK      = 2;
    public static final int    LED_ON         = 3;

    public static final double kLLDistance1   = 48;    // distance from bumper in inches for first reference point
    public static final double kLLVertOffset1 = 0.42;  // LL y reading in degrees for first reference point
    public static final double kLLDistance2   = 60;    // distance from bumper in inches for second reference point
    public static final double kLLVertOffset2 = -4.85; // LL y reading in degrees for second reference point
  }

  public static final class LEDConsts
  {
    public static final int kCANDdleID = 0;

    public enum LEDColor
    {
      LEDCOLOR_OFF,     // CANdle off
      LEDCOLOR_WHITE,   // CANdle white
      LEDCOLOR_RED,     // CANdle red
      LEDCOLOR_ORANGE,  // CANdle orange
      LEDCOLOR_YELLOW,  // CANdle yellow
      LEDCOLOR_GREEN,   // CANdle green
      LEDCOLOR_BLUE,    // CANdle blue
      LEDCOLOR_PURPLE,  // CANdle purple
      LEDCOLOR_DASH     // CANdle color taken from dashboard
    }
  }
}
