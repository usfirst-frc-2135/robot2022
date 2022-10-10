
package frc.robot;

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
  /**
   * public static final class DriveConstants {
   * public static final int kLeftMotor1Port = 0;
   * public static final int kLeftMotor2Port = 1;
   * public static final int kRightMotor1Port = 2;
   * public static final int kRightMotor2Port = 3;
   * }
   */
  public static final class DTConsts
  {

    public static final int kLFDrive1CANID = 1;
    public static final int kLFSteer2CANID = 2;
    public static final int kRFDrive3CANID = 3;
    public static final int kRFSteer4CANID = 4;

    public static final int kLRDrive5CANID = 5;
    public static final int kLRSteer6CANID = 6;
    public static final int kRRDrive7CANID = 7;
    public static final int kRRSteer8CANID = 8;
  }
}
