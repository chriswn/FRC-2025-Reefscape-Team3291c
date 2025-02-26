// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePivotSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  public DutyCycleEncoder intakeEncoder;
  public DigitalInput IntakeLimitSwitch;
  public ProfiledPIDController profiledPIDController;
  public SparkMax pivotMotor;
  public TrapezoidProfile.Constraints trapezoidConstraints;
  public ArmFeedforward armFeedforward;

 
  public enum PivotTarget {
    NONE,
    GROUND,
    STOW,
    MIDLEVELS,
    TOPLEVEL
  }

  public PivotTarget pivot_target = PivotTarget.STOW;

  private double pivotKp = Constants.Intake.PID.kp;
  private double pivotKi = Constants.Intake.PID.ki;
  private double pivotKd = Constants.Intake.PID.kd;
  private double pivotKs = Constants.Intake.pivotKs;
  private double pivotKg = Constants.Intake.pivotKg;
  private double pivotKv = Constants.Intake.pivotKv;
  private double pivotKa = Constants.Intake.pivotKa;

  private double pivotMaxAcceleration = Constants.Intake.pivotMaxAcceleration;
  private double pivotMaxVelocity = Constants.Intake.pivotMaxVelocity;

  public double groundAngle = Constants.Intake.groundAngle;
  public double stowAngle = Constants.Intake.stowAngle;
  public double midLevelsAngle = Constants.Intake.midLevelsAngle;
  public double topLevelAngle = Constants.Intake.topLevelAngle;

  public double encoderOffset = Constants.Intake.pivotEncoderOffset;

  public IntakePivotSubsystem() {
    if (!Preferences.containsKey("encoderOffset")) {
      Preferences.initDouble("encoderOffset", encoderOffset);
    }
    if (!Preferences.containsKey("pivotAngleGround")) {
      Preferences.initDouble("pivotAngleGround", groundAngle);
    }
    if (!Preferences.containsKey("pivotAngleStow")) {
        Preferences.initDouble("pivotAngleStow", stowAngle);
    }
    if (!Preferences.containsKey("pivotAngleMidLevels")) {
        Preferences.initDouble("pivotAngleMidLevels", midLevelsAngle);
    }
    if (!Preferences.containsKey("pivotAngleTopLevel")) {
        Preferences.initDouble("pivotAngleTopLevel", topLevelAngle);
    }

    if (!Preferences.containsKey("pivotKp")) {
        Preferences.initDouble("pivotKp", pivotKp);
    }
    
    if (!Preferences.containsKey("pivotKi")) {
        Preferences.initDouble("pivotKi", pivotKi);
    }
    
    if (!Preferences.containsKey("pivotKd")) {
        Preferences.initDouble("pivotKd", pivotKd);
    }
    
    if (!Preferences.containsKey("pivotKs")) {
        Preferences.initDouble("pivotKs", pivotKs);
    }
    
    if (!Preferences.containsKey("pivotKg")) {
        Preferences.initDouble("pivotKg", pivotKg);
    }
    
    if (!Preferences.containsKey("pivotKv")) {
        Preferences.initDouble("pivotKv", pivotKv);
    }
    
    if (!Preferences.containsKey("pivotKa")) {
        Preferences.initDouble("pivotKa", pivotKa);
    }
    
    if (!Preferences.containsKey("pivotMaxAcceleration")) {
        Preferences.initDouble("pivotMaxAcceleration", pivotMaxAcceleration);
    }
    
    if (!Preferences.containsKey("pivotMaxVelocity")) {
        Preferences.initDouble("pivotMaxVelocity", pivotMaxVelocity);
    }


    this.intakeEncoder = new DutyCycleEncoder(Constants.Intake.encoderID);
    this.intakeEncoder.setInverted(true);

    this.trapezoidConstraints = new TrapezoidProfile.Constraints(pivotMaxVelocity, pivotMaxAcceleration);
    this.profiledPIDController = new ProfiledPIDController(pivotKp, pivotKi, pivotKd, this.trapezoidConstraints);
    this.profiledPIDController.setTolerance(Constants.Intake.tolerance);
    this.armFeedforward = new ArmFeedforward(Constants.Intake.pivotKs, Constants.Intake.pivotKg, Constants.Intake.pivotKv, Constants.Intake.pivotKa);

    this.pivotMotor = new SparkMax(Constants.Intake.PivotID, SparkLowLevel.MotorType.kBrushless);
    this.pivotMotor.setInverted(false);
  }
   
  public void loadPreferences() {
    if (Preferences.getDouble("encoderOffset", encoderOffset) != encoderOffset) {
      encoderOffset = Preferences.getDouble("encoderOffset", encoderOffset);
    }
    if (Preferences.getDouble("pivotAngleGround", groundAngle) != groundAngle) {
      groundAngle = Preferences.getDouble("pivotAngleGround", groundAngle);
    }
    if (Preferences.getDouble("pivotAngleStow", stowAngle) != stowAngle) {
      stowAngle = Preferences.getDouble("pivotAngleStow", stowAngle);
    }
    if (Preferences.getDouble("pivotAngleMidLevels", midLevelsAngle) != midLevelsAngle) {
      midLevelsAngle = Preferences.getDouble("pivotAngleMidLevels", midLevelsAngle);
    }
    if (Preferences.getDouble("pivotAngleTopLevel", topLevelAngle) != topLevelAngle) {
      topLevelAngle = Preferences.getDouble("pivotAngleTopLevel", topLevelAngle);
    }
    if (Preferences.getDouble("pivotMaxAcceleration", pivotMaxAcceleration) != pivotMaxAcceleration || Preferences.getDouble("pivotMaxVelocity", pivotMaxVelocity) != pivotMaxVelocity) {
      pivotMaxAcceleration = Preferences.getDouble("pivotMaxAcceleration", pivotMaxAcceleration);
      pivotMaxVelocity = Preferences.getDouble("pivotMaxVelocity", pivotMaxVelocity);
      trapezoidConstraints = new TrapezoidProfile.Constraints(pivotMaxVelocity, pivotMaxAcceleration);
      profiledPIDController = new ProfiledPIDController(pivotKp, pivotKi, pivotKd, trapezoidConstraints);
    }
    if (Preferences.getDouble("pivotKa", pivotKa) != pivotKa || Preferences.getDouble("pivotKv", pivotKv) != pivotKv || Preferences.getDouble("pivotKg", pivotKg) != pivotKg || Preferences.getDouble("pivotKs", pivotKs) != pivotKs) {
      pivotKa = Preferences.getDouble("pivotKa", pivotKa);
      pivotKv = Preferences.getDouble("pivotKv", pivotKv);
      pivotKg = Preferences.getDouble("pivotKg", pivotKg);
      pivotKs = Preferences.getDouble("pivotKs", pivotKs);
      armFeedforward = new ArmFeedforward(pivotKs, pivotKg, pivotKv, pivotKa);
    }
    if (Preferences.getDouble("pivotKp", pivotKp) != pivotKp) {
      pivotKp = Preferences.getDouble("pivotKp", pivotKp);
      profiledPIDController.setP(pivotKp);
    }
    if (Preferences.getDouble("pivotKi", pivotKi) != pivotKi) {
      pivotKp = Preferences.getDouble("pivotKi", pivotKi);
      profiledPIDController.setI(pivotKi);
    }
    if (Preferences.getDouble("pivotKd", pivotKd) != pivotKd) {
      pivotKp = Preferences.getDouble("pivotKd", pivotKd);
      profiledPIDController.setD(pivotKd);
    }
  }

  public double giveVoltage(double pivotAngle, double current_angle) {
    // pivot control
    SmartDashboard.putNumber("originalAngle", current_angle);
   
    //double angle = Math.abs(1 - current_angle); //reverses it
    double angle = current_angle;
    SmartDashboard.putNumber("updatedAngle", angle);

    double pivotVoltage = profiledPIDController.calculate(angle, pivotAngle) + armFeedforward.calculate(Math.toRadians(angle * 360.0 + 40.0), profiledPIDController.getSetpoint().velocity);
    SmartDashboard.putNumber("intake setpoint", profiledPIDController.getSetpoint().position);

    //double adjustedpivotVoltage = 10 - Math.abs(pivotVoltage);
    double adjustedpivotVoltage = pivotVoltage; //error reversed for voltage
    if (!intakeEncoder.isConnected()) {
      adjustedpivotVoltage = 0.0;
    }

    return adjustedpivotVoltage;
  }
 
  public void stopIntake() {
    pivotMotor.set(0);
  }

  public double pivotTargetToAngle(PivotTarget target) {
    switch (target) {
      case GROUND:
        return groundAngle;
      case STOW:
        return stowAngle;
      case MIDLEVELS:
        return midLevelsAngle;
      case TOPLEVEL:
        return topLevelAngle;
      default:
        // "Safe" default
        return stowAngle;
    }
  }

  public double getCurrentAngle() {
    double value = intakeEncoder.get();
     value += encoderOffset;
    if (value > 1) {
      value %= 1;
    }
    return value;
  }

   //check if at angle
  public boolean ifAtAngle(double targetAngle) {
    boolean value = false;
    if (getCurrentAngle() < targetAngle + Constants.Intake.angleDeadband && getCurrentAngle() > targetAngle - Constants.Intake.angleDeadband) {
      value = true;
    }
    return value;
  }

  public void goToPosition(PivotTarget pivotTarget) {
    double pivotAngle = pivotTargetToAngle(pivotTarget);
    SmartDashboard.putNumber("pivotAngle", pivotAngle);
    double voltage = giveVoltage(pivotAngle, getCurrentAngle());
    if (pivotTarget == PivotTarget.STOW && ifAtAngle(Constants.Intake.stowAngle)) {
      pivotMotor.set(0);
      SmartDashboard.putNumber("getIntakeVoltage", 0);
      return;
    }
    pivotMotor.setVoltage(voltage);
    SmartDashboard.putNumber("getIntakeVoltage", voltage);
  }

  @Override
  public void periodic() {
    loadPreferences();//to be commented out
    goToPosition(pivot_target);
    SmartDashboard.putString("pivot target", pivot_target.toString());
    SmartDashboard.putNumber("pivot encoder reading", getCurrentAngle());
    SmartDashboard.putNumber("pivot adjusted encoder reading", getCurrentAngle() * 360.0);
    SmartDashboard.putBoolean("atAngleGround", ifAtAngle(Constants.Intake.groundAngle));
    SmartDashboard.putBoolean("atAngleStow", ifAtAngle(Constants.Intake.stowAngle));
    SmartDashboard.putBoolean("atAngleMidLevels", ifAtAngle(Constants.Intake.midLevelsAngle));
    SmartDashboard.putBoolean("atAngleTopLevel", ifAtAngle(Constants.Intake.topLevelAngle));
  }
}
 





