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
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;





public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  public DutyCycleEncoder elevatorEncoder;
  public DigitalInput elevatorLimitSwitch;
  public PIDController pidController;
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


  SparkMaxConfig followerConfig;
  SparkMaxConfig leaderConfig;

  public ElevatorSubsystem() {
    

    Preferences.initDouble("elevatorkp", elevatorkp);
    Preferences.initDouble("elevatorki", elevatorki);
    Preferences.initDouble("elevatorkd", elevatorkd);

    this.elevatorEncoder = new DutyCycleEncoder(Constants.Elevator.encoderID);
    this.elevatorLimitSwitch = new DigitalInput(Constants.Elevator.topLimitSwitchID);


    // this.pidController.setD(Constants.Elevator.elevatorPID.kd);
    // SendableRegistry.addChild("D", d);
    // this.pidController.setI(Constants.Elevator.elevatorPID.ki);
    // this.pidController.setP(Constants.Elevator.elevatorPID.kp);


    this.pidController.enableContinuousInput(0, 360);
   


    this.elevatorMotorLeader = new SparkMax(Constants.Elevator.motorFollowerID, SparkLowLevel.MotorType.kBrushless);
    this.elevatorMotorFollower = new SparkMax(Constants.Elevator.motorLeadID, SparkLowLevel.MotorType.kBrushless);
    followerConfig = new SparkMaxConfig();
    followerConfig.follow(elevatorMotorLeader);
    
    this.elevatorMotorFollower.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    this.trapezoidConstraints = 
      new TrapezoidProfile.Constraints(Constants.Elevator.maxVelocity, Constants.Elevator.maxAcceleration);

    this.goal = new TrapezoidProfile.State();
    this.setpoint = new TrapezoidProfile.State();

    this.elevatorFeedforward = new ElevatorFeedforward(Constants.Elevator.ks, Constants.Elevator.kg, Constants.Elevator.kv, Constants.Elevator.ka);

    this.profiledPIDController = new ProfiledPIDController(Constants.Elevator.PID.kp, Constants.Elevator.PID.ki, Constants.Elevator.PID.kd, this.trapezoidConstraints);
    this.profiledPIDController.setGoal(0);
  }


  public void loadPreferences() {
    if (Preferences.getDouble("elevatorkp", elevatorkp) != elevatorkp) {
      elevatorkp = Preferences.getDouble("elevatorkp", elevatorkp);
      pidController.setP(elevatorkp);
    }
    if (Preferences.getDouble("elevatorki", elevatorki) != elevatorki) {
      elevatorkp = Preferences.getDouble("elevatorki", elevatorki);
      pidController.setI(elevatorki);
    }
    if (Preferences.getDouble("elevatorkd", elevatorkd) != elevatorkd) {
      elevatorkp = Preferences.getDouble("elevatorkd", elevatorkd);
      pidController.setD(elevatorkd);
    }
  }

  public double giveVoltage(TrapezoidProfile.State desired_height, double current_height) {
    // Floor control
    SmartDashboard.putNumber("originalheight", current_height);
   
    //double height = Math.abs(360 - current_height); //reverses it
    double height = current_height;
    SmartDashboard.putNumber("updatedheight", height);

    double elevator_floor_voltage = profiledPIDController.calculate(height, desired_height) + elevatorFeedforward.calculate(profiledPIDController.getSetpoint().velocity);

    // If the Floor is at exactly 0.0, it's probably not connected, so disable it
    SmartDashboard.putNumber("pid output", elevator_floor_voltage);
    System.out.println("error: " + elevator_floor_voltage);

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

    System.out.println("final voltage: " + adjustedElevatorFloorVoltage);
    return adjustedElevatorFloorVoltage;
  }
 
  public void stopElevator() {
    elevatorMotorLeader.set(0);
  }

  public double floorTargetToheight(FloorTarget target) {
    switch (target) {
      case GROUND_FLOOR:
        return Constants.Elevator.groundFloor;
      case TOP_FLOOR:
        return Constants.Elevator.topFloor;
      default:
        // "Safe" default
        return 180;
    }
  }

   //check if at floor
  public boolean elevatorAtGroundFloor() {
    boolean value = false;
    if (elevatorEncoder.get() < Constants.Elevator.groundFloor + Constants.Elevator.groundFloor && elevatorEncoder.get() > Constants.Elevator.groundFloor - Constants.Elevator.groundFloor) {
      value = true;
    }
    return value;
  }
  public boolean elevatorAtTopFloor() {
    boolean value = false;
    if (elevatorEncoder.get() < Constants.Elevator.topFloor + Constants.Elevator.topFloor && elevatorEncoder.get() > Constants.Elevator.topFloor - Constants.Elevator.topFloor) {
      value = true;
    }
    return value;
  }


  public void goToGround() {
    //  if (getElevatorHasNote()) {
    //   goToStow();
    // }
    floor_target = FloorTarget.GROUND_FLOOR;
    double desired_height = Constants.Elevator.groundFloor;
    goal = new TrapezoidProfile.State(desired_height, 0);

    System.out.println("stow height target: " + desired_height);
    System.out.println("final voltage: " + giveVoltage(goal, elevatorEncoder.get()) + "\n\n");
    double voltage = giveVoltage(goal, elevatorEncoder.get());
    if (elevatorEncoder.get() < 100 && voltage == -Constants.Elevator.maxVoltage) {
      elevatorMotorLeader.setVoltage(Constants.Elevator.maxVoltage);
    }
    else if (elevatorEncoder.get() > 300) {
      elevatorMotorLeader.setVoltage(-Constants.Elevator.maxVoltage);


    } else {
      elevatorMotorLeader.setVoltage(voltage);
    }
   
    SmartDashboard.putNumber("getVoltage", voltage);
    System.out.println("s");
  }






  @Override
  public void periodic() {
    loadPreferences();
    //This method will be called once per scheduler run
    SmartDashboard.putNumber("encoder reading", elevatorEncoder.get());
    SmartDashboard.putBoolean("elevatorAtGroundFloor", elevatorAtGroundFloor());
    SmartDashboard.putBoolean("elevatorAtTopFloor", elevatorAtTopFloor());

  }
}
 





