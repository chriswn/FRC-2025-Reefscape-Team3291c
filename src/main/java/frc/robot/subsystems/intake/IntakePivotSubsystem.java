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

  public DutyCycleEncoder IntakeEncoder;
  public DigitalInput IntakeLimitSwitch;
  public ProfiledPIDController profiledPIDController;
  public SparkMax pivotMotor;
  public TrapezoidProfile.Constraints trapezoidConstraints;
  public ArmFeedforward armFeedforward;

 
  public enum PivotTarget {
    NONE,
    GROUND,
    SOURCE,
    AMP,
    STOW
  }

  public PivotTarget pivot_target = PivotTarget.STOW;

  private double Intakekp = Constants.Intake.PID.kp;
  private double Intakeki = Constants.Intake.PID.ki;
  private double Intakekd = Constants.Intake.PID.kd;
  private double Intakeks = Constants.Intake.ks;
  private double Intakekg = Constants.Intake.kg;
  private double Intakekv = Constants.Intake.kv;
  private double Intakeka = Constants.Intake.ka;

  private double IntakeMaxAcceleration = Constants.Intake.maxAcceleration;
  private double IntakeMaxVelocity = Constants.Intake.maxVelocity;


  public IntakePivotSubsystem() {
    if (!Preferences.containsKey("Intakekp")) {
      Preferences.initDouble("Intakekp", Intakekp);
  }
  
  if (!Preferences.containsKey("Intakeki")) {
      Preferences.initDouble("Intakeki", Intakeki);
  }
  
  if (!Preferences.containsKey("Intakekd")) {
      Preferences.initDouble("Intakekd", Intakekd);
  }
  
  if (!Preferences.containsKey("Intakeks")) {
      Preferences.initDouble("Intakeks", Intakeks);
  }
  
  if (!Preferences.containsKey("Intakekg")) {
      Preferences.initDouble("Intakekg", Intakekg);
  }
  
  if (!Preferences.containsKey("Intakekv")) {
      Preferences.initDouble("Intakekv", Intakekv);
  }
  
  if (!Preferences.containsKey("Intakeka")) {
      Preferences.initDouble("Intakeka", Intakeka);
  }
  
  if (!Preferences.containsKey("IntakeMaxAccleration")) {
      Preferences.initDouble("IntakeMaxAccleration", IntakeMaxAcceleration);
  }
  
  if (!Preferences.containsKey("IntakeMaxVelocity")) {
      Preferences.initDouble("IntakeMaxVelocity", IntakeMaxVelocity);
  }
  


    this.IntakeEncoder = new DutyCycleEncoder(Constants.Intake.encoderID);
    this.IntakeEncoder.setInverted(false);

    this.trapezoidConstraints = new TrapezoidProfile.Constraints(Constants.Intake.maxVelocity, Constants.Intake.maxAcceleration);
    this.profiledPIDController = new ProfiledPIDController(Constants.Intake.PID.kp, Constants.Intake.PID.ki, Constants.Intake.PID.kd, this.trapezoidConstraints);
    this.profiledPIDController.setTolerance(Constants.Intake.tolerance);
    this.armFeedforward = new ArmFeedforward(Constants.Intake.ks, Constants.Intake.kg, Constants.Intake.kv, Constants.Intake.ka);

    this.pivotMotor = new SparkMax(Constants.Intake.PivotID, SparkLowLevel.MotorType.kBrushless);
  }
   
  public void loadPreferences() {
    if (Preferences.getDouble("IntakeMaxAccleration", IntakeMaxAcceleration) != IntakeMaxAcceleration || Preferences.getDouble("IntakeMaxVelocity", IntakeMaxVelocity) != IntakeMaxVelocity) {
      IntakeMaxAcceleration = Preferences.getDouble("IntakeMaxAccleration", IntakeMaxAcceleration);
      IntakeMaxVelocity = Preferences.getDouble("IntakeMaxVelocity", IntakeMaxVelocity);
      trapezoidConstraints = new TrapezoidProfile.Constraints(IntakeMaxVelocity, IntakeMaxAcceleration);
      profiledPIDController = new ProfiledPIDController(Intakekp, Intakeki, Intakekd, trapezoidConstraints);
    }
    if (Preferences.getDouble("Intakeka", Intakeka) != Intakeka || Preferences.getDouble("Intakekv", Intakekv) != Intakekv || Preferences.getDouble("Intakekg", Intakekg) != Intakekg || Preferences.getDouble("Intakeks", Intakeks) != Intakeks) {
      Intakeka = Preferences.getDouble("Intakeka", Intakeka);
      Intakekv = Preferences.getDouble("Intakekv", Intakekv);
      Intakekg = Preferences.getDouble("Intakekg", Intakekg);
      Intakeks = Preferences.getDouble("Intakeks", Intakeks);
      armFeedforward = new ArmFeedforward(Intakeks, Intakekg, Intakekv, Intakeka);
    }
    if (Preferences.getDouble("Intakekp", Intakekp) != Intakekp) {
      Intakekp = Preferences.getDouble("Intakekp", Intakekp);
      profiledPIDController.setP(Intakekp);
    }
    if (Preferences.getDouble("Intakeki", Intakeki) != Intakeki) {
      Intakekp = Preferences.getDouble("Intakeki", Intakeki);
      profiledPIDController.setI(Intakeki);
    }
    if (Preferences.getDouble("Intakekd", Intakekd) != Intakekd) {
      Intakekp = Preferences.getDouble("Intakekd", Intakekd);
      profiledPIDController.setD(Intakekd);
    }
  }

  public double giveVoltage(double pivotAngle, double current_angle) {
    // Pivot control
    SmartDashboard.putNumber("originalAngle", current_angle);
   
    //double angle = Math.abs(360 - current_angle); //reverses it
    double angle = current_angle;
    SmartDashboard.putNumber("updatedAngle", angle);

    double intakePivotVoltage = profiledPIDController.calculate(angle, pivotAngle) + armFeedforward.calculate(Math.toRadians(angle), profiledPIDController.getSetpoint().velocity);
    SmartDashboard.putNumber("intake setpoint", profiledPIDController.getSetpoint().position);

    //double adjustedIntakePivotVoltage = 10 - Math.abs(intakePivotVoltage);
    double adjustedIntakePivotVoltage = intakePivotVoltage; //error reversed for voltage
    if (!IntakeEncoder.isConnected()) {
      adjustedIntakePivotVoltage = 0.0;
    }

    return adjustedIntakePivotVoltage;
  }
 
  public void stopIntake() {
    pivotMotor.set(0);
  }

  public double pivotTargetToAngle(PivotTarget target) {
    switch (target) {
      case GROUND:
        return Constants.Intake.groundAngle;
      case SOURCE:
        return Constants.Intake.sourceAngle;
      case AMP:
        return Constants.Intake.ampAngle;
      case STOW:
        return Constants.Intake.stowAngle;
      default:
        // "Safe" default
        return 180;
    }
  }

  public double getCurrentAngle() {
    double value = IntakeEncoder.get();
     value += Constants.Intake.pivotEncoderOffset;
    if (value > 1) {
      value %= 1;
    }
    return value;
  }

   //check if at angle
  public boolean ampAtAngle() {
    boolean value = false;
    if (getCurrentAngle() < Constants.Intake.ampAngle + Constants.Intake.angleDeadband && getCurrentAngle() > Constants.Intake.ampAngle - Constants.Intake.angleDeadband) {
      value = true;
    }
    return value;
  }
  public boolean groundAtAngle() {
    boolean value = false;
    if (getCurrentAngle() < Constants.Intake.groundAngle + Constants.Intake.angleDeadband && getCurrentAngle() > Constants.Intake.groundAngle - Constants.Intake.angleDeadband) {
      value = true;
    }
    return value;
  }
  public boolean stowAtAngle() {
    boolean value = false;
    if (getCurrentAngle() < Constants.Intake.stowAngle + Constants.Intake.angleDeadband && getCurrentAngle() > Constants.Intake.stowAngle - Constants.Intake.angleDeadband) {
      value = true;
    }
    return value;
  }
  public boolean sourceAtAngle() {
    boolean value = false;
    if (getCurrentAngle() < Constants.Intake.sourceAngle + Constants.Intake.angleDeadband && getCurrentAngle() > Constants.Intake.sourceAngle - Constants.Intake.angleDeadband) {
      value = true;
    }
    return value;
  }

  public void goToPosition(PivotTarget pivotTarget) {
    double pivotAngle = pivotTargetToAngle(pivotTarget);
    double voltage = giveVoltage(pivotAngle, getCurrentAngle());
    pivotMotor.setVoltage(voltage);
    SmartDashboard.putNumber("getIntakeVoltage", voltage);
  }

  @Override
  public void periodic() {
    loadPreferences();//to be commented out
    SmartDashboard.putNumber("intakePivot encoder reading", getCurrentAngle());
    SmartDashboard.putNumber("intakePivot adjusted encoder reading", getCurrentAngle() * 360.0);
    SmartDashboard.putBoolean("atAngleGround", groundAtAngle());
    SmartDashboard.putBoolean("atAngleAmp", ampAtAngle());
    SmartDashboard.putBoolean("atAngleSource", sourceAtAngle());
    SmartDashboard.putBoolean("atAngleStow", stowAtAngle());
  }
}
 





