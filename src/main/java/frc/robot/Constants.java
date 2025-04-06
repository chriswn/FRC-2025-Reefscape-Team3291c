// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.math.util.Units.degreesToRadians;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class Vision {
      public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
    new Translation3d(0.5, 0.0, 0.5), // Example values - measure your actual camera position
    new Rotation3d(0.0, 0.0, 0.0)
); 
public static final Transform3d TAG_TO_GOAL = new Transform3d(
  new Translation3d(1.5, 0.0, 0.0),
  new Rotation3d(0.0, 0.0, Math.PI)
);


      public static final int TARGET_TAG_ID = 6;
      public static final double CAMERA_HEIGHT = 0.5; // meters
       public static final double TAG_HEIGHT = 1.0; // meters
        // public static final String kCameraName = "YOUR CAMERA NAME";
        // // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

                public static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

// PID Constants (tune these per robot)
public static final double AUTO_ALIGN_kP = 0.8;
public static final double AUTO_ALIGN_kI = 0.0;
public static final double AUTO_ALIGN_kD = 0.1;
        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        // public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        // public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.7, 0.7, Units.degreesToRadians(30));
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(10));
    
    }
    // public static final double CAMERA_HEIGHT_METERS = 0.5;
    // public static final double TAG_HEIGHT_METERS = 1.0;
    // public static final int DESIRED_TAG_ID = 12;

  // Add vision constants here or in an appropriate nested class:
  public static final double VISION_TURN_kP = 0.01; // Adjust this proportional gain as needed
  public static final double MAX_ANGULAR_SPEED = Math.PI; // Example value (radians per second)
  
  public static final int runMotorID = 39;
  public static final int smartCurrentLimit = 30;

  public static final double ROBOT_MASS = (148.0 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10.0; // seconds
  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6.0;
  }

  
  public static class Elevator {
    public static final double deadband = 0.02;

    public static final double maxVelocity = 5.5;
    public static final double maxAcceleration = 5;
    public static final double ks = 0.0;
    public static final double kg = 0.138;
    public static final double kv = 1.78;
    public static final double ka = 0.0;

    public static final double tolerance = 0.05;

    public static class PID {
      public static final double kp = 5.0;
      public static final double ki = 0.0;
      public static final double kd = 0.0;
    }

    //ids
    public static final int encoderAID = 1;
    public static final int encoderBID = 2;
    public static final int motorLeadID = 15;
    public static final int motorFollowerID = 14;
    public static final int topLimitSwitchID = 9;
    public static final int bottomLimitSwitchID = 8;

    public static final double encoderTicksPerRotation = 2048.0;

    //floor pos
    //inch measurements
    public static final double distanceOffGround = 15.0 + (3.0/8.0);
    public static final double distanceToLevel2 = 23.0 + (7.0/8.0) - distanceOffGround;
    public static final double distanceToLevel3 = 40.0 + (4.0/8.0) - distanceOffGround;
    public static final double distanceToLevel4 = 70.0 - distanceOffGround;

    public static final double maxHeightInches = 81.0 + (1.0/4.0) - distanceOffGround;
    public static final double maxHeightRotation = 6.0 + (250.0/360.0);
    public static final double inchesToRotations = maxHeightRotation/maxHeightInches;

    public static final double groundFloor = 0.0;
    public static final double algaeFloor = 6 * inchesToRotations;
    public static final double secondFloor = 1.1;
    public static final double thirdFloor = distanceToLevel3 * inchesToRotations;
    public static final double fourthFloor = distanceToLevel4 * inchesToRotations;
    public static final double topFloor = 6.0 + (250.0/360.0);

    public static final double heightOfHeadBang = 0.88;
    public static final double algaeOffset = 1.0;
  }


  public static class Intake {
    public static final double pivotMaxVelocity = 0.5;
    public static final double pivotMaxAcceleration = 0.5;
    public static final double pivotKs = 0.0;
    public static final double pivotKg = 0.5;
    public static final double pivotKv = 7.3;
    public static final double pivotKa = 0.0;

    public static final double intakeMotorKp = 0.00001;
    public static final double intakeMotorKi = 0.0;
    public static final double intakeMotorKd = 0.0;
    public static final double intakeMotorKs = 0.15;
    public static final double intakeMotorKv = 0.002114;

    public static class PID {
      public static final double kp = 6.0;
      public static final double ki = 0.0;
      public static final double kd = 0.0;
    }

    //ids
    public static final int encoderID = 0; //Changed due to change
    public static final int IntakeID = 23;//21
    public static final int PivotID = 27;//19 
    public static final int intakeLimitSwitchID = 9;
    public static final int CANrangeID = 7;


    public static final double pivotEncoderOffset = 0.715;

    //angles
    public static final double stowAngle = 21.0/360.0;
    public static final double midLevelsFallingOffset = 3.0/360.0;
    public static final double midLevelsAngle = 0.13;//35.0/360.0 - midLevelsFallingOffset;
    public static final double topLevelAngle = 90.0/360.0;
    public static final double groundAngle = 180.0/360.0;//for algae
    public static final double angleToAvoidHeadBang = 0.13;
    public static final double algaeAngle = 0.51;


    public static final double angleDeadband = 5.0/360.0;
    public static final double tolerance = angleDeadband;

    public static final boolean reverseIntakeMotor = true;

    public static final double eSpitSpeed = -1200.0;
    public static final double ejectSpeed = 1600.0;
    public static final double intakeSpeed = 1000.0;
    
    public static final double distanceSensorPointBlankRange = 0.1;

    public static final double intakeTimeToStop = 0.0025;
    public static final double ejectTimeToStop = 0.052;
  }
  

  public final static class Lighting {
    public final static int lightingPort = 2;
    public final static int m_ledBuffer = 200;

    public static enum Colors {
      RAINBOWRAINBOW("Rainbow Palette", -0.99),
      RAINBOWPARTY("Rainbow Party Palette", -0.97),
      RAINBOWOCEAN("Rainbow Ocean Palette", -0.95),
      RAINBOWLAVE("Rainbow Lave Palette", -0.93),
      RAINBOWFOREST("Rainbow Forest Palette", -0.91),
      RAINBOWGLITTER("Rainbow with Glitter", -0.89),
      CONFETTI("Confetti", -0.87),
      SHOTRED("Shot Red", -0.85),
      SHOTBLUE("Shot Blue", -0.83),
      SHOTWHITE("Shot White", -0.81),
      SINELONRAINBOW("Sinelon Rainbow Palette", -0.79),
      SINELONPARTY("Sinelon Party Palette", -0.77),
      SINELONOCEAN("Sinelon Ocean Palette", -0.75),
      SINELONLAVA("Sinelon Lava Palette", -0.73),
      SINELONFOREST("Sinelon Forest Palette", -0.71),
      BEATSRAINBOWPALETTE("Beats per Minute Rainbow Palette", -0.69),
      BEATSPARTYPALETTE("Beats per Minute Party Pallette", -0.67),
      BEATSOCEANPALETTE("Beats per Minute Ocean Pallette", -0.65),
      BEATSLAVAPALETTE("Beats per Minute Lava Pallette", -0.63),
      BEATSFORESTPALETTE("Beats per Minute Forest Pallette", -0.61),
      FIREMEDIUM("Fire Medium", -0.59),
      FIRELARGE("Fire Large", -0.57),
      TWINKLESRAINBOW("Twinkles Rainbow Palette", -0.55),
      TWINKLESPARTY("Twinkles Party Palette", -0.53),
      TWINKLESOCEAN("Twinkles Ocean Palette", -0.51),
      TWINKLESLAVA("Twinkles Lava Palette", -0.49),
      TWINKLESFOREST("Twinkles Forest Palette", -0.47),
      COLORWAVESRAINBOW("Color Waves Rainbow Palette", -0.45),
      COLORWAVESPARTY("Color Waves Party Palette", -0.43),
      COLORWAVESOCEAN("Color Waves Ocean Palette", -0.41),
      COLORWAVESLAVA("Color Waves Lava Palette", -0.39),
      COLORWAVESFOREST("Color Waves Forest Palette", -0.37),
      LARSONRED("Larson Scanner Red", -0.35),
      LARSONGRAY("Larson Scanner Gray", -0.33),
      CHASERED("Light Chase Red", -0.31),
      CHASEBLUE("Light Chase Blue", -0.29),
      CHASEGRAY("Light Chase Gray", -0.27),
      HEARTBEATRED("Heartbeat Red", -0.25),
      HEARTBEATBLUE("Heartbeat Blue", -0.23),
      HEARTBEATWHITE("Heartbeat White", -0.21),
      HEARTBEATGRAY("Heartbeat Gray", -0.19),
      BREATHRED("Breath Red", -0.17),
      BREATHBLUE("Breath Blue", -0.15),
      BREATHGRAY("Breath Gray", -0.13),
      STROBERED("Strobe Red", -0.11),
      STROBEBLUE("Strobe Blue", -0.09),
      STROBEGOLD("Strobe Gold", -0.07),
      STROBEWHITE("Strobe White", -0.05),
      ENDTOOFF("End to End Blend to Off", -0.03),
      LARSONSCANNER("Larson Scanner", -0.01),
      LIGHTCHASE("Light Chase", 0.01),
      HEARTBEATSLOW("Heartbeat Slow", 0.03),
      HEARTBEATMEDIUM("Heartbeat Medium", 0.05),
      HEARTBEATFAST("Heartbeat Fast", 0.07),
      BREATHSLOW("Breath Slow", 0.09),
      BREATHFAST("Breath Fast", 0.11),
      SHOT("Shot", 0.13),
      STROBE("Strobe", 0.15),
      ENDTOOFFTWO("End to End Blend to Off Two", 0.17),
      LARSONSCANNERTWO("Larson Scanner Two", 0.19),
      LIGHTCHASETWO("Light Chase Two", 0.21),
      HEARTBEATSLOWTWO("Heartbeat Slow Two", 0.23),
      HEARTBEATMEDIUMTWO("Heartbeat Medium Two", 0.25),
      HEARTBEATFASTTWO("Heartbeat Fast Two", 0.27),
      BREATHSLOWYTWO("Breath Slow Two", 0.29),
      BREATHFASTTWO("Breath Slow Two", 0.31),
      SHOTTWO("Shot Two", 0.33),
      STROBETWO("Strobe Two", 0.35),
      SPARKLEONEANDTWO("Sparkle Color One on Color Two", 0.37),
      SPARKLETWOANDONE("Sparkle Color Two on Color One", 0.39),
      COLORGRADIENTONEANDTWO("Color Gradient Color One and Two", 0.41),
      BEATSCOLORONEANDTWO("Beats per Minute Color One and Two", 0.43),
      ENDTOENDOFF("End to End Blend Color One and Two", 0.45),
      ENDTOEND("End to End Blend", 0.47),
      COLORONEANDCOLORTWO("Color Ome and Color Two no blending", 0.49),
      TWINKLESCOLORS("Twinkles Color One and Two", 0.51),
      COLORWAVESONEANDTWO("Color Waves Color One and Two", 0.53),
      SINELONONEANDTWO("Sinelon Color One and Two", 0.55),
      HOTPINK("Hot Pink", 0.57),
      DARKRED("Dark Red", 0.59),
      RED("Red", 0.61),
      REDORANGE("Red Orange", 0.63),
      ORANGE("Orange", 0.65),
      GOLD("Gold", 0.67),
      YELLOW("Yellow", 0.69),
      LAWNGREEN("Lawn Green", 0.71),
      LIME("Lime", 0.73),
      DARKGREEN("Dark Green", 0.75),
      GREEN("Green", 0.77),
      BLUEGREEN("Blue Green", 0.79),
      AQUA("Aqua", 0.81),
      SKYBLUE("Sky Blue", 0.83),
      DARKBLUE("Dark Blue", 0.85),
      BLUE("Blue", 0.87),
      BLUEVIOLET("Blue Violet", 0.89),
      VIOLET("Violet", 0.91),
      WHITE("White", 0.93),
      GRAY("Gray", 0.95),
      DARKGRAY("Dark Gray", 0.97),
      OFF("Off", 0.99);

      public class ColorConstants {
        public static final double RAINBOWRAINBOW = -0.99;
        public static final double RAINBOWPARTY = -0.97;
        public static final double RAINBOWOCEAN = -0.95;
        public static final double RAINBOWLAVE = -0.93;
        public static final double RAINBOWFOREST = -0.91;
        public static final double RAINBOWGLITTER = -0.89;
        public static final double CONFETTI = -0.87;
        public static final double SHOTRED = -0.85;
        public static final double SHOTBLUE = -0.83;
        public static final double SHOTWHITE = -0.81;
        public static final double SINELONRAINBOW = -0.79;
        public static final double SINELONPARTY = -0.77;
        public static final double SINELONOCEAN = -0.75;
        public static final double SINELONLAVA = -0.73;
        public static final double SINELONFOREST = -0.71;
        public static final double BEATSRAINBOWPALETTE = -0.69;
        public static final double BEATSPARTYPALETTE = -0.67;
        public static final double BEATSOCEANPALETTE = -0.65;
        public static final double BEATSLAVAPALETTE = -0.63;
        public static final double BEATSFORESTPALETTE = -0.61;
        public static final double FIREMEDIUM = -0.59;
        public static final double FIRELARGE = -0.57;
        public static final double TWINKLESRAINBOW = -0.55;
        public static final double TWINKLESPARTY = -0.53;
        public static final double TWINKLESOCEAN = -0.51;
        public static final double TWINKLESLAVA = -0.49;
        public static final double TWINKLESFOREST = -0.47;
        public static final double COLORWAVESRAINBOW = -0.45;
        public static final double COLORWAVESPARTY = -0.43;
        public static final double COLORWAVESOCEAN = -0.41;
        public static final double COLORWAVESLAVA = -0.39;
        public static final double COLORWAVESFOREST = -0.37;
        public static final double LARSONRED = -0.35;
        public static final double LARSONGRAY = -0.33;
        public static final double CHASERED = -0.31;
        public static final double CHASEBLUE = -0.29;
        public static final double CHASEGRAY = -0.27;
        public static final double HEARTBEATRED = -0.25;
        public static final double HEARTBEATBLUE = -0.23;
        public static final double HEARTBEATWHITE = -0.21;
        public static final double HEARTBEATGRAY = -0.19;
        public static final double BREATHRED = -0.17;
        public static final double BREATHBLUE = -0.15;
        public static final double BREATHGRAY = -0.13;
        public static final double STROBERED = -0.11;
        public static final double STROBEBLUE = -0.09;
        public static final double STROBEGOLD = -0.07;
        public static final double STROBEWHITE = -0.05;
        public static final double ENDTOOFF = -0.03;
        public static final double LARSONSCANNER = -0.01;
        public static final double LIGHTCHASE = 0.01;
        public static final double HEARTBEATSLOW = 0.03;
        public static final double HEARTBEATMEDIUM = 0.05;
        public static final double HEARTBEATFAST = 0.07;
        public static final double BREATHSLOW = 0.09;
        public static final double BREATHFAST = 0.11;
        public static final double SHOT = 0.13;
        public static final double STROBE = 0.15;
        public static final double ENDTOOFFTWO = 0.17;
        public static final double LARSONSCANNERTWO = 0.19;
        public static final double LIGHTCHASETWO = 0.21;
        public static final double HEARTBEATSLOWTWO = 0.23;
        public static final double HEARTBEATMEDIUMTWO = 0.25;
        public static final double HEARTBEATFASTTWO = 0.27;
        public static final double BREATHSLOWYTWO = 0.29;
        public static final double BREATHFASTTWO = 0.31;
        public static final double SHOTTWO = 0.33;
        public static final double STROBETWO = 0.35;
        public static final double SPARKLEONEANDTWO = 0.37;
        public static final double SPARKLETWOANDONE = 0.39;
        public static final double COLORGRADIENTONEANDTWO = 0.41;
        public static final double BEATSCOLORONEANDTWO = 0.43;
        public static final double ENDTOENDOFF = 0.45;
        public static final double ENDTOEND = 0.47;
        public static final double COLORONEANDCOLORTWO = 0.49;
        public static final double TWINKLESCOLORS = 0.51;
        public static final double COLORWAVESONEANDTWO = 0.53;
        public static final double SINELONONEANDTWO = 0.55;
        public static final double HOTPINK = 0.57;
        public static final double DARKRED = 0.59;
        public static final double RED = 0.61;
        public static final double REDORANGE = 0.63;
        public static final double ORANGE = 0.65;
        public static final double GOLD = 0.67;
        public static final double YELLOW = 0.69;
        public static final double LAWNGREEN = 0.71;
        public static final double LIME = 0.73;
        public static final double DARKGREEN = 0.75;
        public static final double GREEN = 0.77;
        public static final double BLUEGREEN = 0.79;
        public static final double AQUA = 0.81;
        public static final double SKYBLUE = 0.83;
        public static final double DARKBLUE = 0.85;
        public static final double BLUE = 0.87;
        public static final double BLUEVIOLET = 0.89;
        public static final double VIOLET = 0.91;
        public static final double WHITE = 0.93;
        public static final double GRAY = 0.95;
        public static final double DARKGRAY = 0.97;
        public static final double OFF = 0.99;
      }

      public final String colorName;
      public final double colorValue;

      Colors(String colorName, double colorValue) {
        this.colorName = colorName;
        this.colorValue = colorValue;
      }

      public String getColorName() {
        return this.colorName;
      }

      public double getColorValue() {
        return this.colorValue;
      }
    }

    public final static Colors startingColor = Colors.OFF;
    public final static Colors disableColor = Colors.RAINBOWOCEAN;
  }

  public static Object ColorChanger;

  public static class ButtonList {
    public static final int a = 1;
    public static final int b = 2; 
    public static final int x = 3; 
    public static final int y = 4; 
    public static final int lb = 5; 
    public static final int rb = 6; 
    public static final int back = 7; 
    public static final int start = 8; 
    public static final int l3 = 9; 
    public static final int r3 = 10; 

  }

     public static class VisionConstants {

    /** Physical location of the apriltag camera on the robot, relative to the center of the robot. */
    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT = new Transform3d(
        new Translation3d(-0.06, 0.2, -0.2127),
        new Rotation3d(0.0, degreesToRadians(15.0), degreesToRadians(-3.0)));

    /** Physical location of the shooter camera on the robot, relative to the center of the robot. */
    public static final Transform3d LOW_LIMELIGHT_TO_ROBOT = new Transform3d(
        new Translation3d(-0.083, 0.254, -0.537),
        new Rotation3d(0.0, degreesToRadians(-9.0), degreesToRadians(-1.0)));

    public static final String LOW_LIMELIGHT_NAME = "limelight";
    
    /** Physical location of the high camera on the robot, relative to the center of the robot. */
    public static final Transform3d HIGH_LIMELIGHT_TO_ROBOT = new Transform3d(
        new Translation3d(-0.11, -0.015, -0.895),
        new Rotation3d(degreesToRadians(-90.0), degreesToRadians(34.6), 0.0));

    public static final String HIGH_LIMELIGHT_NAME = "limelight-high";
    
    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;

    // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose to the opposite alliance
    public static final Pose2d FLIPPING_POSE = new Pose2d(
        new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
        new Rotation2d(Math.PI));

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  }


}