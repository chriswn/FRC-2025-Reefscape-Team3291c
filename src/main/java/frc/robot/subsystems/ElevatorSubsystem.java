// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakePivotSubsystem.PivotTarget;


public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  public Encoder elevatorEncoder;
  public DigitalInput topElevatorLimitSwitch;
  public DigitalInput bottomElevatorLimitSwitch;

  public SparkMax elevatorMotorLeader;
  public SparkMax elevatorMotorFollower;
  public TrapezoidProfile.Constraints trapezoidConstraints;
  public ElevatorFeedforward elevatorFeedforward;
  public ProfiledPIDController profiledPIDController;
  public TrapezoidProfile.State goal;

  public Boolean algaeMode = false;

  public enum FloorTarget {
    NONE,
    GROUND_FLOOR,
    SECOND_FLOOR,
    THIRD_FLOOR,
    FOURTH_FLOOR,
    TOP_FLOOR
  }

  // Input: Desired state
  public FloorTarget floor_target = FloorTarget.GROUND_FLOOR;

  // Output: Motor set values
  double secondFloor = Constants.Elevator.secondFloor;
  double thirdFloor = Constants.Elevator.thirdFloor;
  double fourthFloor = Constants.Elevator.fourthFloor;

  private double elevatorkp = Constants.Elevator.PID.kp;
  private double elevatorki = Constants.Elevator.PID.ki;
  private double elevatorkd = Constants.Elevator.PID.kd;
  private double elevatorks = Constants.Elevator.ks;
  private double elevatorkg = Constants.Elevator.kg;
  private double elevatorkv = Constants.Elevator.kv;
  private double elevatorka = Constants.Elevator.ka;

  private double elevatorMaxAcceleration = Constants.Elevator.maxAcceleration;
  private double elevatorMaxVelocity = Constants.Elevator.maxVelocity;

  private double algaeOffset = Constants.Elevator.algaeOffset;

  SparkMaxConfig followerConfig;
  SparkMaxConfig leaderConfig;

  public ElevatorSubsystem() {
    if (!Preferences.containsKey("algaeOffset")) {
      Preferences.initDouble("algaeOffset", algaeOffset);
    }
    if (!Preferences.containsKey("elevatorSecondFloorHeight")) {
        Preferences.initDouble("elevatorSecondFloorHeight", secondFloor);
    }
    if (!Preferences.containsKey("elevatorThirdFloorHeight")) {
        Preferences.initDouble("elevatorThirdFloorHeight", thirdFloor);
    }
    if (!Preferences.containsKey("elevatorFourthFloorHeight")) {
        Preferences.initDouble("elevatorFourthFloorHeight", fourthFloor);
    }
    if (!Preferences.containsKey("elevatorkp")) {
      Preferences.initDouble("elevatorkp", elevatorkp);
  }
  
  if (!Preferences.containsKey("elevatorki")) {
      Preferences.initDouble("elevatorki", elevatorki);
  }
  
  if (!Preferences.containsKey("elevatorkd")) {
      Preferences.initDouble("elevatorkd", elevatorkd);
  }
  
  if (!Preferences.containsKey("elevatorks")) {
      Preferences.initDouble("elevatorks", elevatorks);
  }
  
  if (!Preferences.containsKey("elevatorkg")) {
      Preferences.initDouble("elevatorkg", elevatorkg);
  }
  
  if (!Preferences.containsKey("elevatorkv")) {
      Preferences.initDouble("elevatorkv", elevatorkv);
  }
  
  if (!Preferences.containsKey("elevatorka")) {
      Preferences.initDouble("elevatorka", elevatorka);
  }
  
  if (!Preferences.containsKey("elevatorMaxAccleration")) {
      Preferences.initDouble("elevatorMaxAccleration", elevatorMaxAcceleration);
  }
  
  if (!Preferences.containsKey("elevatorMaxVelocity")) {
      Preferences.initDouble("elevatorMaxVelocity", elevatorMaxVelocity);
  }

    this.elevatorEncoder = new Encoder(Constants.Elevator.encoderAID, Constants.Elevator.encoderBID, false, Encoder.EncodingType.k2X);//may need reversing
    this.topElevatorLimitSwitch = new DigitalInput(Constants.Elevator.topLimitSwitchID);
    this.topElevatorLimitSwitch = new DigitalInput(Constants.Elevator.bottomLimitSwitchID);

    this.elevatorMotorLeader = new SparkMax(Constants.Elevator.motorLeadID, SparkLowLevel.MotorType.kBrushless);
    this.elevatorMotorLeader.setInverted(true);

    this.elevatorMotorFollower = new SparkMax(Constants.Elevator.motorFollowerID, SparkLowLevel.MotorType.kBrushless);
    this.followerConfig = new SparkMaxConfig();
    this.followerConfig.follow(elevatorMotorLeader);
    this.followerConfig.inverted(true);
    this.elevatorMotorFollower.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    this.trapezoidConstraints = 
      new TrapezoidProfile.Constraints(Constants.Elevator.maxVelocity, Constants.Elevator.maxAcceleration);

    this.goal = new TrapezoidProfile.State();

    this.elevatorFeedforward = new ElevatorFeedforward(Constants.Elevator.ks, Constants.Elevator.kg, Constants.Elevator.kv, Constants.Elevator.ka);

    this.profiledPIDController = new ProfiledPIDController(Constants.Elevator.PID.kp, Constants.Elevator.PID.ki, Constants.Elevator.PID.kd, this.trapezoidConstraints);
    this.profiledPIDController.setGoal(0);
    this.profiledPIDController.setTolerance(Constants.Elevator.tolerance);
  }


  public void loadPreferences() {
    if (Preferences.getDouble("algaeOffset", algaeOffset) != algaeOffset) {
      algaeOffset = Preferences.getDouble("algaeOffset", algaeOffset);
    }
    if (Preferences.getDouble("elevatorSecondFloorHeight", secondFloor) != secondFloor) {
      secondFloor = Preferences.getDouble("elevatorSecondFloorHeight", secondFloor);
    }
    if (Preferences.getDouble("elevatorThirdFloorHeight", thirdFloor) != thirdFloor) {
      thirdFloor = Preferences.getDouble("elevatorThirdFloorHeight", thirdFloor);
    }
    if (Preferences.getDouble("elevatorFourthFloorHeight", fourthFloor) != fourthFloor) {
      fourthFloor = Preferences.getDouble("elevatorFourthFloorHeight", fourthFloor);
    }
    if (Preferences.getDouble("elevatorMaxAccleration", elevatorMaxAcceleration) != elevatorMaxAcceleration || Preferences.getDouble("elevatorMaxVelocity", elevatorMaxVelocity) != elevatorMaxVelocity) {
      elevatorMaxAcceleration = Preferences.getDouble("elevatorMaxAccleration", elevatorMaxAcceleration);
      elevatorMaxVelocity = Preferences.getDouble("elevatorMaxVelocity", elevatorMaxVelocity);
      trapezoidConstraints = new TrapezoidProfile.Constraints(elevatorMaxVelocity, elevatorMaxAcceleration);
      profiledPIDController = new ProfiledPIDController(elevatorkp, elevatorki, elevatorkd, trapezoidConstraints);
    }
    if (Preferences.getDouble("elevatorka", elevatorka) != elevatorka || Preferences.getDouble("elevatorkv", elevatorkv) != elevatorkv || Preferences.getDouble("elevatorkg", elevatorkg) != elevatorkg || Preferences.getDouble("elevatorks", elevatorks) != elevatorks) {
      elevatorka = Preferences.getDouble("elevatorka", elevatorka);
      elevatorkv = Preferences.getDouble("elevatorkv", elevatorkv);
      elevatorkg = Preferences.getDouble("elevatorkg", elevatorkg);
      elevatorks = Preferences.getDouble("elevatorks", elevatorks);
      elevatorFeedforward = new ElevatorFeedforward(elevatorks, elevatorkg, elevatorkv, elevatorka);
    }
    if (Preferences.getDouble("elevatorkp", elevatorkp) != elevatorkp) {
      elevatorkp = Preferences.getDouble("elevatorkp", elevatorkp);
      profiledPIDController.setP(elevatorkp);
    }
    if (Preferences.getDouble("elevatorki", elevatorki) != elevatorki) {
      elevatorkp = Preferences.getDouble("elevatorki", elevatorki);
      profiledPIDController.setI(elevatorki);
    }
    if (Preferences.getDouble("elevatorkd", elevatorkd) != elevatorkd) {
      elevatorkp = Preferences.getDouble("elevatorkd", elevatorkd);
      profiledPIDController.setD(elevatorkd);
    }
  }

  public double giveVoltage(double height_in_ticks) {
    // Floor control
    double height = height_in_ticks / Constants.Elevator.encoderTicksPerRotation;
    SmartDashboard.putNumber("elevator adjusted height", height);

    double elevatorFloorVoltage = profiledPIDController.calculate(height) + elevatorFeedforward.calculate(profiledPIDController.getSetpoint().velocity);
    SmartDashboard.putNumber("elevator setpoint", profiledPIDController.getSetpoint().position);
    SmartDashboard.putNumber("elevator floor voltage", elevatorFloorVoltage);

    return elevatorFloorVoltage;
  }
 
  public void stopElevator() {
    elevatorMotorLeader.set(0);
  }

  public double floorTargetToHeight(FloorTarget target) {
    switch (target) {
      case GROUND_FLOOR:
        return Constants.Elevator.groundFloor; //ground floor and top floor shouldn't be changed
      case SECOND_FLOOR:
        return secondFloor;
      case THIRD_FLOOR:
        return thirdFloor;
      case FOURTH_FLOOR:
        return fourthFloor;
      case TOP_FLOOR:
        return Constants.Elevator.topFloor;
      default:
        // "Safe" default
        return Constants.Elevator.groundFloor;
    }
  }

   //check if at floor
  public boolean ifAtFloor(Double target) {
    boolean value = false;
    if (elevatorEncoder.get()/Constants.Elevator.encoderTicksPerRotation < target + Constants.Elevator.deadband && elevatorEncoder.get()/Constants.Elevator.encoderTicksPerRotation > target - Constants.Elevator.deadband) {
      value = true;
    }
    return value;
  }
 
  public void setTarget(FloorTarget target) {
    floor_target = target;
  }

  public void goToPosition() {
    double desired_height = floorTargetToHeight(floor_target);
    if (algaeMode && (floor_target == FloorTarget.SECOND_FLOOR || floor_target == FloorTarget.THIRD_FLOOR)) {
      desired_height += algaeOffset;
    }
    SmartDashboard.putNumber("elevator_desired_height", desired_height);
    goal = new TrapezoidProfile.State(desired_height, 0);
    profiledPIDController.setGoal(goal);

    double voltage = giveVoltage(elevatorEncoder.get());
    elevatorMotorLeader.setVoltage(voltage);
    SmartDashboard.putNumber("elevatorVoltage", voltage);
  }

  @Override
  public void periodic() {
    goToPosition();
    loadPreferences();//to be commented out
    //This method will be called once per scheduler run
    SmartDashboard.putNumber("elevator encoder reading", elevatorEncoder.get());
    SmartDashboard.putNumber("elevator adjusted encoder reading", elevatorEncoder.get()/Constants.Elevator.encoderTicksPerRotation);

    SmartDashboard.putBoolean("elevatorAtGroundFloor", ifAtFloor(Constants.Elevator.groundFloor));
    SmartDashboard.putBoolean("elevatorAtSecondFloor", ifAtFloor(Constants.Elevator.secondFloor));
    SmartDashboard.putBoolean("elevatorAtThirdFloor", ifAtFloor(Constants.Elevator.thirdFloor));
    SmartDashboard.putBoolean("elevatorAtFourthFloor", ifAtFloor(Constants.Elevator.fourthFloor));

    SmartDashboard.putBoolean("elevatorAtTopFloor", ifAtFloor(Constants.Elevator.topFloor));
    SmartDashboard.putNumber("elevatorFloorTarget", floorTargetToHeight(floor_target));
  }
}
 





