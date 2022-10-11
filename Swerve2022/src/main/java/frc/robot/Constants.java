
package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do not put anything functional in this class.
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
    public static final int    kLFDrive1CANID         = 1;
    public static final int    kLFSteer2CANID         = 2;
    public static final int    kRFDrive3CANID         = 3;
    public static final int    kRFSteer4CANID         = 4;

    public static final int    kLRDrive5CANID         = 5;
    public static final int    kLRSteer6CANID         = 6;
    public static final int    kRRDrive7CANID         = 7;
    public static final int    kRRSteer8CANID         = 8;

    public static final double kWheelDiaMeters        = Units.inchesToMeters(4.0); // 4in (39.37 in/meter)
    public static final double kGearRatio             = 6.75;
    public static final double kEncoderMetersPerCount = (kWheelDiaMeters * Math.PI) / (Falcon500.kEncoderCPR) / kGearRatio;

  }
}
