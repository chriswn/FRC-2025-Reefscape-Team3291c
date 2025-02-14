// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems.intake;


import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
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

  // Input: Desired state
  public PivotTarget pivot_target = PivotTarget.STOW;

  // Output: Motor set values

  private double Intakekp = Constants.Intake.PID.kp;
  private double Intakeki = Constants.Intake.PID.ki;
  private double Intakekd = Constants.Intake.PID.kd;
  private double Intakeks = Constants.Intake.ks;
  private double Intakekg = Constants.Intake.kg;
  private double Intakekv = Constants.Intake.kv;
  private double Intakeka = Constants.Intake.ka;




  public IntakePivotSubsystem() {
    Preferences.initDouble("Intakekp", Intakekp);
    Preferences.initDouble("Intakeki", Intakeki);
    Preferences.initDouble("Intakekd", Intakekd);
    Preferences.initDouble("Intakeks", Intakeks);
    Preferences.initDouble("Intakekg", Intakekg);
    Preferences.initDouble("Intakekv", Intakekv);
    Preferences.initDouble("Intakeka", Intakeka);

    this.IntakeEncoder = new DutyCycleEncoder(Constants.Intake.encoderID);
    this.IntakeEncoder.setInverted(false);
    //this.IntakeLimitSwitch = new DigitalInput(Constants.Intake.intakeLimitSwitchID);

    this.trapezoidConstraints = new TrapezoidProfile.Constraints(Constants.Intake.maxVelocity, Constants.Intake.maxAcceleration);

    this.profiledPIDController = new ProfiledPIDController(Constants.Intake.PID.kp, Constants.Intake.PID.ki, Constants.Intake.PID.kd, this.trapezoidConstraints);
    this.profiledPIDController.setGoal(0);
    this.profiledPIDController.setTolerance(Constants.Intake.tolerance);


    // this.pidController.setD(Constants.Intake.IntakePID.kd);
    // SendableRegistry.addChild("D", d);
    // this.pidController.setI(Constants.Intake.IntakePID.ki);
    // this.pidController.setP(Constants.Intake.IntakePID.kp);


    //this.profiledPIDController.enableContinuousInput(0, 360);

    this.armFeedforward = new ArmFeedforward(Constants.Intake.ks, Constants.Intake.kg, Constants.Intake.kv, Constants.Intake.ka);
   


    this.pivotMotor = new SparkMax(Constants.Intake.PivotID, SparkLowLevel.MotorType.kBrushless);
  }
   
  public void loadPreferences() {
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

    double Intake_pivot_voltage = profiledPIDController.calculate(angle, pivotAngle) + armFeedforward.calculate(Math.toRadians(angle), profiledPIDController.getSetpoint().velocity);
    SmartDashboard.putNumber("intake setpoint", profiledPIDController.getSetpoint().position);

    //double adjustedIntakePivotVoltage = 10 - Math.abs(Intake_pivot_voltage);
    double adjustedIntakePivotVoltage = Intake_pivot_voltage; //error reversed for voltage
    if (!IntakeEncoder.isConnected()) {
      adjustedIntakePivotVoltage = 0.0;
    }
    if (adjustedIntakePivotVoltage > Constants.Intake.maxPivotVoltage) {
      adjustedIntakePivotVoltage = Constants.Intake.maxPivotVoltage;
    }
    if (adjustedIntakePivotVoltage < -Constants.Intake.maxPivotVoltage) {
      adjustedIntakePivotVoltage = -Constants.Intake.maxPivotVoltage;
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
    // value *= 360;
     value += Constants.Intake.k_pivotEncoderOffset;
    // if (value > 360) {
    //   value %= 360;
    // }
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
    loadPreferences();
    //This method will be called once per scheduler run
    SmartDashboard.putNumber("intakePivot encoder reading", getCurrentAngle());
    SmartDashboard.putBoolean("atAngleGround", groundAtAngle());
    SmartDashboard.putBoolean("atAngleAmp", ampAtAngle());
    SmartDashboard.putBoolean("atAngleSource", sourceAtAngle());
    SmartDashboard.putBoolean("atAngleStow", stowAtAngle());
  }
}
 





