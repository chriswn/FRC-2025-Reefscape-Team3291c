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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;





public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  public DutyCycleEncoder elevatorEncoder;
  public DigitalInput elevatorLimitSwitch;
  public PIDController pidController;
  public SparkMax elevatorMotorLeader;
  public SparkMax elevatorMotorFollower;
  public TrapezoidProfile trapezoidProfile;
  public ElevatorFeedforward elevatorFeedforward;
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


    this.pidController = new PIDController(Constants.Elevator.PID.kp, Constants.Elevator.PID.ki, Constants.Elevator.PID.kd);
    // this.pidController.setD(Constants.Elevator.intakePID.kd);
    // SendableRegistry.addChild("D", d);
    // this.pidController.setI(Constants.Elevator.intakePID.ki);
    // this.pidController.setP(Constants.Elevator.intakePID.kp);


    this.pidController.enableContinuousInput(0, 360);
   


    this.elevatorMotorLeader = new SparkMax(Constants.Elevator.motorFollowerID, SparkLowLevel.MotorType.kBrushless);
    this.elevatorMotorFollower = new SparkMax(Constants.Elevator.motorLeadID, SparkLowLevel.MotorType.kBrushless);
    followerConfig = new SparkMaxConfig();
    followerConfig.follow(elevatorMotorLeader);
    
    this.elevatorMotorFollower.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    this.trapezoidProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(Constants.Elevator.maxVelocity, Constants.Elevator.maxAcceleration)
      );

    this.goal = new TrapezoidProfile.State();
    this.setpoint = new TrapezoidProfile.State();

    this.elevatorFeedforward = new ElevatorFeedforward(Constants.Elevator.ks, Constants.Elevator.kg, Constants.Elevator.kv, Constants.Elevator.ka);
    this.trapezoidProfile.calculate(elevatorkd, setpoint, goal);//move this immediately
    leaderConfig = new SparkMaxConfig();
    leaderConfig.();
    goal = new TrapezoidProfile.State(5, 0);
    pidController.setSetpoint(
        elevatorEncoder.get(),
        setpoint.position,
        elevatorFeedforward.calculate(setpoint.velocity) / 12.0);
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

  public double giveVoltage(double floor_angle, double current_angle) {
    // Floor control
    SmartDashboard.putNumber("originalAngle", current_angle);
   
    //double angle = Math.abs(360 - current_angle); //reverses it
    double angle = current_angle;
    SmartDashboard.putNumber("updatedAngle", angle);

    double intake_floor_voltage = pidController.calculate(angle, floor_angle);

    // If the Floor is at exactly 0.0, it's probably not connected, so disable it
    SmartDashboard.putNumber("pid output", intake_floor_voltage);
    System.out.println("error: " + intake_floor_voltage);

    //double adjustedIntakefloorVoltage = 10 - Math.abs(intake_floor_voltage);
    double adjustedIntakeFloorVoltage = intake_floor_voltage; //error reversed for voltage
    if (elevatorEncoder.get() == 0.0) {
      adjustedIntakeFloorVoltage = 0.0;
    }
    if (adjustedIntakeFloorVoltage > Constants.Elevator.maxVoltage) {
      adjustedIntakeFloorVoltage = Constants.Elevator.maxVoltage;
    }
    if (adjustedIntakeFloorVoltage < -Constants.Elevator.maxVoltage) {
      adjustedIntakeFloorVoltage = -Constants.Elevator.maxVoltage;
    }

    System.out.println("final voltage: " + adjustedIntakeFloorVoltage);
    return adjustedIntakeFloorVoltage;
  }
 
  public void stopIntake() {
    elevatorMotorLeader.set(0);
  }

  public double floorTargetToAngle(FloorTarget target) {
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
    //  if (getIntakeHasNote()) {
    //   goToStow();
    // }
    floor_target = FloorTarget.GROUND_FLOOR;
    double floor_angle = Constants.Elevator.groundFloor;
    System.out.println("stow angle target: " + floor_angle);
    System.out.println("final voltage: " + giveVoltage(floor_angle, elevatorEncoder.get()) + "\n\n");
    double voltage = giveVoltage(floor_angle, elevatorEncoder.get());
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
 





