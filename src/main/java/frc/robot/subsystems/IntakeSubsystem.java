// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;


import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ColorChanger;
import frc.robot.subsystems.PreferencesSubsystem;




public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  public DutyCycleEncoder intakeEncoder;
  public DigitalInput intakeLimitSwitch;
  public PIDController pidController;
  public CANSparkMax pivotMotor;
  public ColorChanger colorChanger;
  public PreferencesSubsystem preferencesSubsystem;
 
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

  private double intakekp = Constants.intake.intakePID.kp;
  private double intakeki = Constants.intake.intakePID.ki;
  private double intakekd = Constants.intake.intakePID.kd;




  public IntakeSubsystem(ColorChanger colorChanger, PreferencesSubsystem preferencesSubsystem) {
    Preferences.initDouble("intakekp", intakekp);
    Preferences.initDouble("intakeki", intakeki);
    Preferences.initDouble("intakekd", intakekd);


    this.colorChanger = colorChanger;
    this.preferencesSubsystem = preferencesSubsystem;
    this.intakeEncoder = new DutyCycleEncoder(Constants.intake.encoderID);
    this.intakeLimitSwitch = new DigitalInput(Constants.intake.intakeLimitSwitchID);


    this.pidController = new PIDController(Constants.intake.intakePID.kp, Constants.intake.intakePID.ki, Constants.intake.intakePID.kd);
    // this.pidController.setD(Constants.intake.intakePID.kd);
    // SendableRegistry.addChild("D", d);
    // this.pidController.setI(Constants.intake.intakePID.ki);
    // this.pidController.setP(Constants.intake.intakePID.kp);


    this.pidController.enableContinuousInput(0, 360);
   


    this.pivotMotor = new CANSparkMax(Constants.intake.PivotID, CANSparkLowLevel.MotorType.kBrushless);
  }
   
  public void loadPreferences() {
    if (Preferences.getDouble("intakekp", intakekp) != intakekp) {
      intakekp = Preferences.getDouble("intakekp", intakekp);
      pidController.setP(intakekp);
    }
    if (Preferences.getDouble("intakeki", intakeki) != intakeki) {
      intakekp = Preferences.getDouble("intakeki", intakeki);
      pidController.setI(intakeki);
    }
    if (Preferences.getDouble("intakekd", intakekd) != intakekd) {
      intakekp = Preferences.getDouble("intakekd", intakekd);
      pidController.setD(intakekd);
    }
  }
  public double giveVoltage(double pivot_angle, double current_angle) {
    // Pivot control
    SmartDashboard.putNumber("originalAngle", current_angle);
   
    //double angle = Math.abs(360 - current_angle); //reverses it
    double angle = current_angle;
    SmartDashboard.putNumber("updatedAngle", angle);

    double intake_pivot_voltage = pidController.calculate(angle, pivot_angle);

    // If the pivot is at exactly 0.0, it's probably not connected, so disable it
    SmartDashboard.putNumber("pid output", intake_pivot_voltage);
    System.out.println("error: " + intake_pivot_voltage);

    //double adjustedIntakePivotVoltage = 10 - Math.abs(intake_pivot_voltage);
    double adjustedIntakePivotVoltage = intake_pivot_voltage; //error reversed for voltage
    if (intakeEncoder.get() == 0.0) {
      adjustedIntakePivotVoltage = 0.0;
    }
    if (adjustedIntakePivotVoltage > preferencesSubsystem.maxPivotVoltage) {
      adjustedIntakePivotVoltage = preferencesSubsystem.maxPivotVoltage;
    }
    if (adjustedIntakePivotVoltage < -preferencesSubsystem.maxPivotVoltage) {
      adjustedIntakePivotVoltage = -preferencesSubsystem.maxPivotVoltage;
    }
   if (!ampAtAngle() && !sourceAtAngle() && !stowAtAngle() && !groundAtAngle()) {
    //adjustedIntakePivotVoltage += Math.cos(167 - angle) * Constants.intake.intakePID.kcos; //feedforward
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
        return preferencesSubsystem.groundAngle;
      case SOURCE:
        return preferencesSubsystem.sourceAngle;
      case AMP:
        return preferencesSubsystem.ampAngle;
      case STOW:
        return preferencesSubsystem.stowAngle;
      default:
        // "Safe" default
        return 180;
    }
  }


  public double getCurrentAngle() {
    double value = intakeEncoder.getAbsolutePosition();
    value *= 360;
    value += Constants.intake.k_pivotEncoderOffset;
    if (value > 360) {
      value %= 360;
    }
    return value;
  }

  public boolean getIntakeHasNote() {
    //must be inverted
    return !intakeLimitSwitch.get();
  }
   //check if at angle
  public boolean ampAtAngle() {
    boolean value = false;
    if (getCurrentAngle() < preferencesSubsystem.ampAngle + Constants.angleDeadband && getCurrentAngle() > preferencesSubsystem.ampAngle - Constants.angleDeadband) {
      value = true;
    }
    return value;
  }
  public boolean groundAtAngle() {
    boolean value = false;
    if (getCurrentAngle() < preferencesSubsystem.groundAngle + Constants.angleDeadband && getCurrentAngle() > preferencesSubsystem.groundAngle - Constants.angleDeadband) {
      value = true;
    }
    return value;
  }
  public boolean stowAtAngle() {
    boolean value = false;
    if (getCurrentAngle() < preferencesSubsystem.stowAngle + Constants.angleDeadband && getCurrentAngle() > preferencesSubsystem.stowAngle - Constants.angleDeadband) {
      value = true;
    }
    return value;
  }
  public boolean sourceAtAngle() {
    boolean value = false;
    if (getCurrentAngle() < preferencesSubsystem.sourceAngle + Constants.angleDeadband && getCurrentAngle() > preferencesSubsystem.sourceAngle - Constants.angleDeadband) {
      value = true;
    }
    return value;
  }


  public void goToGround() {
    //  if (getIntakeHasNote()) {
    //   goToStow();
    // }
    pivot_target = PivotTarget.GROUND;
    double pivot_angle = preferencesSubsystem.groundAngle;
    System.out.println("stow angle target: " + pivot_angle);
    System.out.println("final voltage: " + giveVoltage(pivot_angle, getCurrentAngle()) + "\n\n");
    double voltage = giveVoltage(pivot_angle, getCurrentAngle());
    if (getCurrentAngle() < 100 && voltage == -preferencesSubsystem.maxPivotVoltage) {
      pivotMotor.setVoltage(preferencesSubsystem.maxPivotVoltage);
    }
    else if (getCurrentAngle() > 300) {
      pivotMotor.setVoltage(-preferencesSubsystem.maxPivotVoltage);


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
   if (getCurrentAngle() > 90 && voltage == preferencesSubsystem.maxPivotVoltage) {
      pivotMotor.setVoltage(-preferencesSubsystem.maxPivotVoltage);
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
    SmartDashboard.putBoolean("limit switch", getIntakeHasNote());
    //This method will be called once per scheduler run
    if (!getIntakeHasNote() && getCurrentAngle() < preferencesSubsystem.stowAngle + Constants.angleDeadband && getCurrentAngle() > preferencesSubsystem.stowAngle - Constants.angleDeadband) {
      colorChanger.setRed();
    }
    else if (getIntakeHasNote() && getCurrentAngle() < preferencesSubsystem.stowAngle + Constants.angleDeadband && getCurrentAngle() > preferencesSubsystem.stowAngle - Constants.angleDeadband) {
      colorChanger.setGreen();
    }
    else if (getIntakeHasNote()) {
      colorChanger.setBlue();
    }
    SmartDashboard.putNumber("encoder reading", getCurrentAngle());
    SmartDashboard.putBoolean("atAngleGround", groundAtAngle());
    SmartDashboard.putBoolean("atAngleAmp", ampAtAngle());
    SmartDashboard.putBoolean("atAngleSource", sourceAtAngle());
    SmartDashboard.putBoolean("atAngleStow", stowAtAngle());
  }
}
 





