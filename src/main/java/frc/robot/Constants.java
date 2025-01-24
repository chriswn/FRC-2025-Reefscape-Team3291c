// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final int kLauncherSubLeftMotorId = 18;
  public static final int kLauncherSubRightMotorId = 14;

  public static final double kLauncherSubP = 0.00005;
  public static final double kLauncherSubI = 0.0;
  public static final double kLauncherSubD = 0.0;
  public static final double kLauncherSubFF = 0.0002;

  public static final double kLauncherSubMinOutput = 0;
  public static final double kLauncherSubMaxOutput = 1;


  public static final int leftClimberID = 24;//24
  public static final int rightClimberID = 15;//15

  public static final double feedSpeed = 0.4;   //0.40
  public static final double launchSpeed = 5000 * 0.65;//5000 * percentage 
  public static final double timeToWait = 1;
  public static final double hangSpeed = 0.5;
  public static final double launcherTargetVoltage = launchSpeed * 12;
  public static final double gracePeriod = 0.3;
  
  public static final int LEDColors = 1;
  public static final double angleDeadband = 5;

  public static class buttonList {
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
  public static class intake {

    public static class intakePID {
    public static final double kp = 0.1;
    public static final double ki = 0.0;
    public static final double kd = 0.0;
    public static final double kcos = 0.5;
    }

    //ids
    public static final int encoderID = 0; //Changed due to change
    public static final int IntakeID = 21;//21
    public static final int PivotID = 19;//19 
    public static final int intakeLimitSwitchID = 9;

    public static final double k_pivotEncoderOffset = 230;

    //angles
    public static final double groundAngle = 167 - -35;//doesn't like 0
    public static final double stowAngle = 167 - 164;
    public static final double sourceAngle = 167 - 55;
    public static final double ampAngle = 167 - 95;

    public static final double maxPivotVoltage = 5;

    public static final double ejectSpeed = 0.5 * 5000;
    public static final double intakeSpeed = 0.7 * 5000;
    public static final int launchNoteTimeInSecs = 1;
    
  }


  public static class Swerve {
   
    public static final double stickDeadband = 0.1;//configure and mess around with later
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW- 

    public static final double trackWidth = Units.inchesToMeters(22.5);
    public static final double wheelBase = Units.inchesToMeters(22.5);
    public static final double wheelDiameter = Units.inchesToMeters(3.9965);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = ((150/7) / 1.0); // 150/7:1

    public static final double kMaxTranslationAcceleration = 3.0; //meters per second squared
    public static final double kMaxStrafeAcceleration = 3.0; //meters per second squared
    public static final double kMaxRotationAcceleration = 3.0; //radians per second squared

    // Kinematics
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            //front left ++ 
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            //front right +-
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            //back left -+
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            //Back right --
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Profiling Values */
    // Meters per second IN THEORY
    public static final double maxSpeed = 4.4196;

    /**
     * Find out?
     * 
     * In theory the max speed is already given, but I can't find the max
     * angular velocity, so we'll probably calculate both anyways
     */
    public static final double maxAngularVelocity = 11.5;

    // Swerve Voltage Compensation
    public static final double voltageComp = 12.0;

    /* Drive Motor Conversion Factors */
    // The conversion factor to multiply the native units by 
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI)/ driveGearRatio;
    public static final double driveConversionPositionFactor1 =
        (wheelDiameter * Math.PI);
    public static final double driveConversionPositionFactor2 =
        (wheelDiameter * Math.PI)/ driveGearRatio*42;

    // The conversion factor to multiply the native units by 
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;

    // The conversion factor to multiply the native units by (angle position)
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    // Swerve Current Limiting
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    // Motor Inverts
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = true;

    // Angle Encoder Invert
    public static final boolean canCoderInvert = false;

    // Angle Motor PID Values
    public static final double angleKP = 0.003;//0.003; //play around with/tune this later 0.00025
    public static final double angleKI = 0;//.0000001;
    public static final double angleKD = 0.00000001;
    public static final double angleKFF = 0;//0.0;

    // Drive Motor PID Values
    public static final double driveKP = 0.09;
    public static final double driveKI = 0.000000001;
    public static final double driveKD = 0.00045;

    //feedforward constants
    public static final double ffkS = 0.667;
    public static final double ffkV = 2.44;
    public static final double ffkA = 0.27;

    /**************************
     * Module Specific Constants
     **************************/
    // Back Right Module - Module 0 
    public static final class Mod0 {
      // SparkMAX CAN Device ID
      public static final int driveMotorID = 10;

      // SparkMAX CAN Device ID
      public static final int angleMotorID = 11;

      // CANCoder CAN Device ID
      public static final int canCoderID = 2;

      // Wheel starting angle offset
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(325.019531250);

      public static final double angleMultiplier = 1.00;

      // Constants in a nice package.
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleMultiplier);
    }

    // Back Left Module - Module 1 
    public static final class Mod1 {
      // SparkMAX CAN Device ID
      public static final int driveMotorID = 6;

      // SparkMAX CAN Device ID
      public static final int angleMotorID = 5;

      // CANCoder CAN Device ID
      public static final int canCoderID = 3;

      // Wheel starting angle offset
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(147.041015625);

      public static final double angleMultiplier = 1.00;

      // Constants in a nice package.
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleMultiplier);
    }

    // Front Right Module - Module 2
    public static final class Mod2 {
      // SparkMAX CAN Device ID
      public static final int driveMotorID = 12;

      // SparkMAX CAN Device ID
      public static final int angleMotorID = 13; //13

      // CANCoder CAN Device ID
      public static final int canCoderID = 1;

      // Wheel starting angle offset
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(174.8144);

      public static final double angleMultiplier = 1.00;

      // Constants in a nice package.
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleMultiplier);
    }

    // Front Left Module - Module 3
    public static final class Mod3 {
      // SparkMAX CAN Device ID
      public static final int driveMotorID = 9;

      // SparkMAX CAN Device ID
      public static final int angleMotorID = 8;

      // CANCoder CAN Device ID
      public static final int canCoderID = 0;

      // Wheel starting angle offset
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(284.326171875);

      public static final double angleMultiplier = 1.00;

      // Constants in a nice package.
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleMultiplier);
    }
  }
  public static final class Vision {
    public static final double XOffset = 0;
    public static final double XRange = 20;
    public static final double XDeadband = 1;
    public static final double XProportionalGain = 2;

    public static final double desiredDistanceSpeakerInches = 37.5 + 6;//37.5 is length of subwoofer from limelight
    //public static final double desiredDistanceAmpInches = 37.5 + 6;//37.5 is length of subwoofer from limelight
    public static final double limelightMountAngleDegrees = 28.8;
    public static final double limelightLensHeightInches = 16.875;
    public static final double goalHeightInches = 57.125;

    public static final double findAprilTagTurnSpeed = 0.1;
    public static final double yTargetValue = 1;

  }

    public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.4196; 
    public static final double kMaxAccelerationMetersPerSecondSquared = 5;
    public static final double kMaxAngularSpeedRadiansPerSecond = 11.5;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 11.5 * 11.5;
    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public final static class Lighting {
    public final static int lightingPort = 2;
    public final static int m_ledBuffer = 200;

    public static enum Colors {
     RAINBOWRAINBOW  ("Rainbow Palette", -0.99),
     RAINBOWPARTY ("Rainbow Party Palette", -0.97),
     RAINBOWOCEAN ("Rainbow Ocean Palette", -0.95),
     RAINBOWLAVE ("Rainbow Lave Palette", -0.93),
     RAINBOWFOREST ("Rainbow Forest Palette", -0.91),
     RAINBOWGLITTER ("Rainbow with Glitter", -0.89),
     CONFETTI ("Confetti", -0.87),
     SHOTRED ("Shot Red", -0.85),
     SHOTBLUE ("Shot Blue", -0.83),
     SHOTWHITE ("Shot White", -0.81),
     SINELONRAINBOW ("Sinelon Rainbow Palette", -0.79),
     SINELONPARTY ("Sinelon Party Palette", -0.77),
     SINELONOCEAN ("Sinelon Ocean Palette", -0.75),
     SINELONLAVA ("Sinelon Lava Palette", -0.73),
     SINELONFOREST ("Sinelon Forest Palette", -0.71),
     BEATSRAINBOWPALETTE ("Beats per Minute Rainbow Palette", -0.69),
     BEATSPARTYPALETTE ("Beats per Minute Party Pallette", -0.67),
     BEATSOCEANPALETTE ("Beats per Minute Ocean Pallette", -0.65),
     BEATSLAVAPALETTE ("Beats per Minute Lava Pallette", -0.63),
     BEATSFORESTPALETTE ("Beats per Minute Forest Pallette", -0.61),
     FIREMEDIUM ("Fire Medium", -0.59),
     FIRELARGE ("Fire Large", -0.57),
     TWINKLESRAINBOW ("Twinkles Rainbow Palette", -0.55),
     TWINKLESPARTY ("Twinkles Party Palette", -0.53),
     TWINKLESOCEAN ("Twinkles Ocean Palette", -0.51),
     TWINKLESLAVA ("Twinkles Lava Palette", -0.49),
     TWINKLESFOREST ("Twinkles Forest Palette", -0.47),
     COLORWAVESRAINBOW ("Color Waves Rainbow Palette", -0.45),
     COLORWAVESPARTY ("Color Waves Party Palette", -0.43),
     COLORWAVESOCEAN ("Color Waves Ocean Palette", -0.41),
     COLORWAVESLAVA ("Color Waves Lava Palette", -0.39),
     COLORWAVESFOREST ("Color Waves Forest Palette", -0.37),
     LARSONRED ("Larson Scanner Red", -0.35),
     LARSONGRAY ("Larson Scanner Gray", -0.33),
     CHASERED ("Light Chase Red", -0.31),
     CHASEBLUE ("Light Chase Blue", -0.29),
     CHASEGRAY ("Light Chase Gray", -0.27),
     HEARTBEATRED ("Heartbeat Red", -0.25),
     HEARTBEATBLUE ("Heartbeat Blue", -0.23),
     HEARTBEATWHITE ("Heartbeat White", -0.21),
     HEARTBEATGRAY ("Heartbeat Gray", -0.19),
     BREATHRED ("Breath Red", -0.17),
     BREATHBLUE ("Breath Blue", -0.15),
     BREATHGRAY ("Breath Gray", -0.13),
     STROBERED ("Strobe Red", -0.11),
     STROBEBLUE ("Strobe Blue", -0.09),
     STROBEGOLD ("Strobe Gold", -0.07),
     STROBEWHITE ("Strobe White", -0.05),
     ENDTOOFF ("End to End Blend to Off", -0.03),
     LARSONSCANNER ("Larson Scanner", -0.01),
     LIGHTCHASE ("Light Chase", 0.01),
     HEARTBEATSLOW ("Heartbeat Slow", 0.03),
     HEARTBEATMEDIUM ("Heartbeat Medium", 0.05),
     HEARTBEATFAST ("Heartbeat Fast", 0.07),
     BREATHSLOW ("Breath Slow", 0.09),
     BREATHFAST ("Breath Fast", 0.11),
     SHOT ("Shot", 0.13),
     STROBE ("Strobe", 0.15),
     ENDTOOFFTWO ("End to End Blend to Off Two", 0.17),
     LARSONSCANNERTWO ("Larson Scanner Two", 0.19),
     LIGHTCHASETWO ("Light Chase Two", 0.21),
     HEARTBEATSLOWTWO ("Heartbeat Slow Two", 0.23),
     HEARTBEATMEDIUMTWO ("Heartbeat Medium Two", 0.25),
     HEARTBEATFASTTWO ("Heartbeat Fast Two", 0.27),
     BREATHSLOWYTWO ("Breath Slow Two", 0.29),
     BREATHFASTTWO ("Breath Slow Two", 0.31),
     SHOTTWO ("Shot Two", 0.33),
     STROBETWO ("Strobe Two", 0.35),
     SPARKLEONEANDTWO ("Sparkle Color One on Color Two", 0.37),
     SPARKLETWOANDONE ("Sparkle Color Two on Color One", 0.39),
     COLORGRADIENTONEANDTWO ("Color Gradient Color One and Two", 0.41),
     BEATSCOLORONEANDTWO ("Beats per Minute Color One and Two", 0.43),
     ENDTOENDOFF ("End to End Blend Color One and Two", 0.45),
     ENDTOEND ("End to End Blend", 0.47),
     COLORONEANDCOLORTWO ("Color Ome and Color Two no blending", 0.49),
     TWINKLESCOLORS ("Twinkles Color One and Two", 0.51),
     COLORWAVESONEANDTWO ("Color Waves Color One and Two", 0.53),
     SINELONONEANDTWO ("Sinelon Color One and Two", 0.55),
     HOTPINK ("Hot Pink", 0.57),
     DARKRED ("Dark Red", 0.59),
     RED ("Red", 0.61),
     REDORANGE ("Red Orange", 0.63),
     ORANGE ("Orange", 0.65),
     GOLD ("Gold", 0.67),
     YELLOW ("Yellow", 0.69),
     LAWNGREEN ("Lawn Green", 0.71),
     LIME ("Lime", 0.73),
     DARKGREEN ("Dark Green", 0.75),
     GREEN ("Green", 0.77),
     BLUEGREEN ("Blue Green", 0.79),
     AQUA ("Aqua", 0.81),
     SKYBLUE ("Sky Blue", 0.83),
     DARKBLUE ("Dark Blue", 0.85),
     BLUE ("Blue", 0.87),
     BLUEVIOLET ("Blue Violet", 0.89),
     VIOLET ("Violet", 0.91),
     WHITE ("White", 0.93),
     GRAY ("Gray", 0.95),
     DARKGRAY ("Dark Gray", 0.97),
     OFF ("Off", 0.99);

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
}