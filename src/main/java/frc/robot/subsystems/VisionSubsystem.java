// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.LimelightHelpers.LimelightTarget_Fiducial;

//import com.revrobotics.CANSparkLowLevel.MotorType;
//import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightHelpers.LimelightTarget_Fiducial;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */

    public double AimkP = .035;
    public double AimkI = 0.0;
    public double AimkD = 0.0007;

    public double dkP = 0.1;
    public double dkI = 0.0;
    public double dkD = 0.0001;

    public PIDController AimvisionPID;
        public PIDController distancePID;

   // public CANSparkMax testMotor;

    public  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

   public LimelightResults limelight = LimelightHelpers.getLatestResults("limelight");

  public VisionSubsystem() {
    //tx is horizontal value of target, ty is vertical value of target, ta is area, ts is skew of target, tv is validity of target

    LimelightHelpers.setPipelineIndex("limelight",0);
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry targetpose_cameraspace = table.getEntry("targetpose_cameraspace");
    
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    // Initializing the angle motor PID Controller with PID values
    this.AimvisionPID = new PIDController(
      AimkP,
      AimkI,
      AimkD
    );

    this.distancePID = new PIDController(
      dkP,
      dkI,
      dkD
    );

  }

public double proportionalAiming() {    
    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 21 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight");
    //double targetingAngularVelocity = table.getEntry("tx").getDouble(0);
    SmartDashboard.putNumber("TX", LimelightHelpers.getTX("limelight"));//checking the value

    targetingAngularVelocity = AimvisionPID.calculate(targetingAngularVelocity);//plug it in to pid 

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  public double getLimelightSpeed() {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  //formula to get the distance from a certain point, to then use in percise movement (you lowkey could just use the formula above but like this works too)
  public double getDistanceToSpeaker() {
    double ty = LimelightHelpers.getTY("limelight");
    double targetOffsetAngle_Vertical = ty;//offset from the crosshair

    double desiredDistanceInches = Constants.Vision.desiredDistanceSpeakerInches;//the subwoofer + 6 inches 

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = Constants.Vision.limelightMountAngleDegrees;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = Constants.Vision.limelightLensHeightInches;

    // distance from the target to the floor
    double goalHeightInches = Constants.Vision.goalHeightInches; //accounts for the lowest edge of the speaker's hood

    //mounting angle plus the offset from the crosshair which should maybe be inverted?
    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    //calculate motor output to go to desired distance smooooothly (little extra might just need the p but like why not)
    double distance = distancePID.calculate(distanceFromLimelightToGoalInches, desiredDistanceInches);
    //return that distance!!!!
    return distance;
  }
  // public double getDistanceToAmp() {
  //   double ty = LimelightHelpers.getTY("limelight");
  //   double targetOffsetAngle_Vertical = ty;//offset from the crosshair

  //   double desiredDistanceInches = Constants.Vision.desiredDistanceSpeakerInches;//the subwoofer + 6 inches 

  //   // how many degrees back is your limelight rotated from perfectly vertical?
  //   double limelightMountAngleDegrees = Constants.Vision.limelightMountAngleDegrees;

  //   // distance from the center of the Limelight lens to the floor
  //   double limelightLensHeightInches = Constants.Vision.limelightLensHeightInches;

  //   // distance from the target to the floor
  //   double goalHeightInches = Constants.Vision.goalHeightInches; //accounts for the lowest edge of the speaker's hood

  //   //mounting angle plus the offset from the crosshair which should maybe be inverted?
  //   double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
  //   double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

  //   //calculate distance
  //   double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
  //   //calculate motor output to go to desired distance smooooothly (little extra might just need the p but like why not)
  //   double distance = distancePID.calculate(distanceFromLimelightToGoalInches, desiredDistanceInches);
  //   //return that distance!!!!
  //   return distance;
  // }

  //gets the TX to then use in the drive to apriltag commands
  public double getTXSwerve() {
    return LimelightHelpers.getTX("limelight");
  }

  public double getTYSwerve() {
    return LimelightHelpers.getTY("limelight");
  }

  //checks if the intended apriltag id has been found
  public boolean isThereATarget() {

    if (LimelightHelpers.getTV("limelight") == true){
      return true;
    }
    else {
      return false;
    }

  }

  public double getApriltagId() {

    double id = LimelightHelpers.getFiducialID("limelight");
    return id;

  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
