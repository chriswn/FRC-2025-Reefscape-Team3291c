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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;





public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  public Encoder elevatorEncoder;
  public DigitalInput elevatorLimitSwitch;
  //public PIDController pidController;
  public SparkMax elevatorMotorLeader;
  public SparkMax elevatorMotorFollower;
  public TrapezoidProfile.Constraints trapezoidConstraints;
  public ElevatorFeedforward elevatorFeedforward;
  public ProfiledPIDController profiledPIDController;
  public TrapezoidProfile.State goal;
  public TrapezoidProfile.State setpoint;

 
  public enum FloorTarget {
    NONE,
    GROUND_FLOOR,
    TOP_FLOOR
  }

  // Input: Desired state
  public FloorTarget floor_target = FloorTarget.GROUND_FLOOR;

  // Output: Motor set values

  private double elevatorkp = Constants.Elevator.PID.kp;
  private double elevatorki = Constants.Elevator.PID.ki;
  private double elevatorkd = Constants.Elevator.PID.kd;
  private double elevatorks = Constants.Elevator.ks;
  private double elevatorkg = Constants.Elevator.kg;
  private double elevatorkv = Constants.Elevator.kv;
  private double elevatorka = Constants.Elevator.ka;


  SparkMaxConfig followerConfig;
  SparkMaxConfig leaderConfig;

  public ElevatorSubsystem() {
    Preferences.initDouble("elevatorkp", elevatorkp);
    Preferences.initDouble("elevatorki", elevatorki);
    Preferences.initDouble("elevatorkd", elevatorkd);
    Preferences.initDouble("elevatorks", elevatorks);
    Preferences.initDouble("elevatorkg", elevatorkg);
    Preferences.initDouble("elevatorkv", elevatorkv);
    Preferences.initDouble("elevatorka", elevatorka);

    this.elevatorEncoder = new Encoder(Constants.Elevator.encoderAID, Constants.Elevator.encoderBID, false, Encoder.EncodingType.k1X);
    this.elevatorLimitSwitch = new DigitalInput(Constants.Elevator.topLimitSwitchID);


    // this.pidController.setD(Constants.Elevator.elevatorPID.kd);
    // SendableRegistry.addChild("D", d);
    // this.pidController.setI(Constants.Elevator.elevatorPID.ki);
    // this.pidController.setP(Constants.Elevator.elevatorPID.kp);


    //this.pidController.enableContinuousInput(0, 360);
   


    this.elevatorMotorLeader = new SparkMax(Constants.Elevator.motorLeadID, SparkLowLevel.MotorType.kBrushless);
    this.elevatorMotorFollower = new SparkMax(Constants.Elevator.motorFollowerID, SparkLowLevel.MotorType.kBrushless);
    this.followerConfig = new SparkMaxConfig();
    this.followerConfig.follow(elevatorMotorLeader);
    
    this.elevatorMotorFollower.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    this.trapezoidConstraints = 
      new TrapezoidProfile.Constraints(Constants.Elevator.maxVelocity, Constants.Elevator.maxAcceleration);

    this.goal = new TrapezoidProfile.State();
    this.setpoint = new TrapezoidProfile.State();

    this.elevatorFeedforward = new ElevatorFeedforward(Constants.Elevator.ks, Constants.Elevator.kg, Constants.Elevator.kv, Constants.Elevator.ka);

    this.profiledPIDController = new ProfiledPIDController(Constants.Elevator.PID.kp, Constants.Elevator.PID.ki, Constants.Elevator.PID.kd, this.trapezoidConstraints);
    this.profiledPIDController.setGoal(0);
    this.profiledPIDController.setTolerance(Constants.Elevator.tolerance);
  }


  public void loadPreferences() {
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

  public double giveVoltage(TrapezoidProfile.State desired_height, double current_height) {
    // Floor control
    SmartDashboard.putNumber("originalheight", current_height);
   
    //double height = Math.abs(360 - current_height); //reverses it
    double height = current_height;
    SmartDashboard.putNumber("updatedheight", height);

    double elevator_floor_voltage = profiledPIDController.calculate(height, desired_height) + elevatorFeedforward.calculate(profiledPIDController.getSetpoint().velocity);
    SmartDashboard.putNumber("elevator setpoint", profiledPIDController.getSetpoint().position);
    SmartDashboard.putNumber("elevator floor voltage", elevator_floor_voltage);
    // If the Floor is at exactly 0.0, it's probably not connected, so disable it

    //double adjustedElevatorfloorVoltage = 10 - Math.abs(elevator_floor_voltage);
    double adjustedElevatorFloorVoltage = elevator_floor_voltage; //error reversed for voltage
    if (elevatorEncoder.get() == 0.0) {
      adjustedElevatorFloorVoltage = 0.0;
    }
    if (adjustedElevatorFloorVoltage > Constants.Elevator.maxVoltage) {
      adjustedElevatorFloorVoltage = Constants.Elevator.maxVoltage;
    }
    if (adjustedElevatorFloorVoltage < -Constants.Elevator.maxVoltage) {
      adjustedElevatorFloorVoltage = -Constants.Elevator.maxVoltage;
    }

    return adjustedElevatorFloorVoltage;
  }
 
  public void stopElevator() {
    elevatorMotorLeader.set(0);
  }

  public double floorTargetToHeight(FloorTarget target) {
    switch (target) {
      case GROUND_FLOOR:
        return Constants.Elevator.groundFloor;
      case TOP_FLOOR:
        return Constants.Elevator.topFloor;
      default:
        // "Safe" default
        return Constants.Elevator.groundFloor;
    }
  }

   //check if at floor
  public boolean elevatorAtGroundFloor() {
    boolean value = false;
    if (elevatorEncoder.get() < Constants.Elevator.groundFloor + Constants.Elevator.deadband && elevatorEncoder.get() > Constants.Elevator.groundFloor - Constants.Elevator.deadband) {
      value = true;
    }
    return value;
  }
  public boolean elevatorAtTopFloor() {
    boolean value = false;
    if (elevatorEncoder.get() < Constants.Elevator.topFloor + Constants.Elevator.deadband && elevatorEncoder.get() > Constants.Elevator.topFloor - Constants.Elevator.deadband) {
      value = true;
    }
    return value;
  }
  public void setTarget(FloorTarget target) {
    floor_target = target;
  }

  public void goToPosition() {
    //  if (getElevatorHasNote()) {
    //   goToStow();
    // }
    double desired_height = floorTargetToHeight(floor_target);
    goal = new TrapezoidProfile.State(desired_height, 0);

    double voltage = giveVoltage(goal, elevatorEncoder.get());

    elevatorMotorLeader.setVoltage(voltage);
    
   
    SmartDashboard.putNumber("getVoltage", voltage);
  }

  @Override
  public void periodic() {
    goToPosition();
    loadPreferences();
    //This method will be called once per scheduler run
    SmartDashboard.putNumber("encoder reading", elevatorEncoder.get());
    SmartDashboard.putBoolean("elevatorAtGroundFloor", elevatorAtGroundFloor());
    SmartDashboard.putBoolean("elevatorAtTopFloor", elevatorAtTopFloor());

  }
}
 





