// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems.intake;


import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.math.controller.PIDController;
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
  public PIDController pidController;
  public SparkMax pivotMotor;

 
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

  private double Intakekp = Constants.Intake.intakePID.kp;
  private double Intakeki = Constants.Intake.intakePID.ki;
  private double Intakekd = Constants.Intake.intakePID.kd;




  public IntakePivotSubsystem() {
    Preferences.initDouble("Intakekp", Intakekp);
    Preferences.initDouble("Intakeki", Intakeki);
    Preferences.initDouble("Intakekd", Intakekd);

    this.IntakeEncoder = new DutyCycleEncoder(Constants.Intake.encoderID);
    this.IntakeLimitSwitch = new DigitalInput(Constants.Intake.intakeLimitSwitchID);


    this.pidController = new PIDController(Constants.Intake.intakePID.kp, Constants.Intake.intakePID.ki, Constants.Intake.intakePID.kd);
    // this.pidController.setD(Constants.Intake.IntakePID.kd);
    // SendableRegistry.addChild("D", d);
    // this.pidController.setI(Constants.Intake.IntakePID.ki);
    // this.pidController.setP(Constants.Intake.IntakePID.kp);


    this.pidController.enableContinuousInput(0, 360);
   


    this.pivotMotor = new SparkMax(Constants.Intake.PivotID, SparkLowLevel.MotorType.kBrushless);
  }
   
  public void loadPreferences() {
    if (Preferences.getDouble("Intakekp", Intakekp) != Intakekp) {
      Intakekp = Preferences.getDouble("Intakekp", Intakekp);
      pidController.setP(Intakekp);
    }
    if (Preferences.getDouble("Intakeki", Intakeki) != Intakeki) {
      Intakekp = Preferences.getDouble("Intakeki", Intakeki);
      pidController.setI(Intakeki);
    }
    if (Preferences.getDouble("Intakekd", Intakekd) != Intakekd) {
      Intakekp = Preferences.getDouble("Intakekd", Intakekd);
      pidController.setD(Intakekd);
    }
  }

  public double giveVoltage(double pivot_angle, double current_angle) {
    // Pivot control
    SmartDashboard.putNumber("originalAngle", current_angle);
   
    //double angle = Math.abs(360 - current_angle); //reverses it
    double angle = current_angle;
    SmartDashboard.putNumber("updatedAngle", angle);

    double Intake_pivot_voltage = pidController.calculate(angle, pivot_angle);

    // If the pivot is at exactly 0.0, it's probably not connected, so disable it
    SmartDashboard.putNumber("pid output", Intake_pivot_voltage);
    System.out.println("error: " + Intake_pivot_voltage);

    //double adjustedIntakePivotVoltage = 10 - Math.abs(Intake_pivot_voltage);
    double adjustedIntakePivotVoltage = Intake_pivot_voltage; //error reversed for voltage
    if (IntakeEncoder.get() == 0.0) {
      adjustedIntakePivotVoltage = 0.0;
    }
    if (adjustedIntakePivotVoltage > Constants.Intake.maxPivotVoltage) {
      adjustedIntakePivotVoltage = Constants.Intake.maxPivotVoltage;
    }
    if (adjustedIntakePivotVoltage < -Constants.Intake.maxPivotVoltage) {
      adjustedIntakePivotVoltage = -Constants.Intake.maxPivotVoltage;
    }
   if (!ampAtAngle() && !sourceAtAngle() && !stowAtAngle() && !groundAtAngle()) {
    //adjustedIntakePivotVoltage += Math.cos(167 - angle) * Constants.Intake.IntakePID.kcos; //feedforward
   }
    System.out.println("final voltage: " + adjustedIntakePivotVoltage);
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
    value *= 360;
    value += Constants.Intake.k_pivotEncoderOffset;
    if (value > 360) {
      value %= 360;
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


  public void goToGround() {
    //  if (getIntakeHasNote()) {
    //   goToStow();
    // }
    pivot_target = PivotTarget.GROUND;
    double pivot_angle = Constants.Intake.groundAngle;
    System.out.println("stow angle target: " + pivot_angle);
    System.out.println("final voltage: " + giveVoltage(pivot_angle, getCurrentAngle()) + "\n\n");
    double voltage = giveVoltage(pivot_angle, getCurrentAngle());
    if (getCurrentAngle() < 100 && voltage == -Constants.Intake.maxPivotVoltage) {
      pivotMotor.setVoltage(Constants.Intake.maxPivotVoltage);
    }
    else if (getCurrentAngle() > 300) {
      pivotMotor.setVoltage(-Constants.Intake.maxPivotVoltage);


    } else {
      pivotMotor.setVoltage(voltage);
    }
   
    SmartDashboard.putNumber("getVoltage", voltage);
    System.out.println("s");
  }


  public void goToSource() {
    pivot_target = PivotTarget.SOURCE;
    double pivot_angle = pivotTargetToAngle(pivot_target);
    System.out.println("stow angle target: " + pivot_angle);
    System.out.println("final voltage: " + giveVoltage(pivot_angle, getCurrentAngle()) + "\n\n");
    double voltage = giveVoltage(pivot_angle, getCurrentAngle());
    pivotMotor.setVoltage(voltage);
    SmartDashboard.putNumber("getVoltage", voltage);
    System.out.println("s");
  }


  public void goToAmp() {
    pivot_target = PivotTarget.AMP;
    double pivot_angle = pivotTargetToAngle(pivot_target);
    System.out.println("stow angle target: " + pivot_angle);
    System.out.println("final voltage: " + giveVoltage(pivot_angle, getCurrentAngle()) + "\n\n");
    double voltage = giveVoltage(pivot_angle, getCurrentAngle());
    pivotMotor.setVoltage(voltage);
    SmartDashboard.putNumber("getVoltage", voltage);
    System.out.println("s");
  }


  public void goToStow() {
    pivot_target = PivotTarget.STOW;
    double pivot_angle = pivotTargetToAngle(pivot_target);
    System.out.println("stow angle target: " + pivot_angle);
   System.out.println("final voltage: " + giveVoltage(pivot_angle, getCurrentAngle()) + "\n\n");
    double voltage = giveVoltage(pivot_angle, getCurrentAngle());
   if (getCurrentAngle() > 90 && voltage == Constants.Intake.maxPivotVoltage) {
      pivotMotor.setVoltage(-Constants.Intake.maxPivotVoltage);
    }
      else {
        pivotMotor.setVoltage(voltage);
      }   
    SmartDashboard.putNumber("getVoltage", voltage);
    System.out.println("s");
  }



  @Override
  public void periodic() {
    loadPreferences();
    //This method will be called once per scheduler run
    SmartDashboard.putNumber("encoder reading", getCurrentAngle());
    SmartDashboard.putBoolean("atAngleGround", groundAtAngle());
    SmartDashboard.putBoolean("atAngleAmp", ampAtAngle());
    SmartDashboard.putBoolean("atAngleSource", sourceAtAngle());
    SmartDashboard.putBoolean("atAngleStow", stowAtAngle());
  }
}
 





