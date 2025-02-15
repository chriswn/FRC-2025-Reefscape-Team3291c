package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;

public class IntakeMotorSubsystem extends SubsystemBase {

  /*-------------------------------- public instance variables ---------------------------------*/
  public SparkMax IntakeMotorMotor;

  public SparkClosedLoopController IntakeMotorPID;

  public RelativeEncoder IntakeMotorEncoder;

  public SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1000);
  public SparkMaxConfig config;

  public CANrange canRange;

  public double intakeMotorKp = Preferences.getDouble("intakeMotorKp", Constants.Intake.kLauncherSubP);
  public double intakeMotorKi = Preferences.getDouble("intakeMotorKi", Constants.Intake.kLauncherSubI);
  public double intakeMotorKd = Preferences.getDouble("intakeMotorKd", Constants.Intake.kLauncherSubD);
  public double intakeMotorKff = Preferences.getDouble("intakeMotorKff", Constants.Intake.kLauncherSubFF);
  public double intakeSpeed = Preferences.getDouble("intakeSpeed", Constants.Intake.intakeSpeed);
  public double ejectSpeed = Preferences.getDouble("ejectSpeed", Constants.Intake.ejectSpeed);




  public IntakeMotorSubsystem() {
    canRange = new CANrange(0);
    if (!Preferences.containsKey("intakeSpeed")) {
      Preferences.initDouble("intakeSpeed", intakeSpeed);
    }
    if (!Preferences.containsKey("intakeMotorKp")) {
      Preferences.initDouble("intakeMotorKp", Constants.Intake.kLauncherSubP);
    }
    if (!Preferences.containsKey("intakeMotorKi")) {
      Preferences.initDouble("intakeMotorKi", Constants.Intake.kLauncherSubI);
    }
    if (!Preferences.containsKey("intakeMotorKd")) {
      Preferences.initDouble("intakeMotorKd", Constants.Intake.kLauncherSubD);
    }
    if (!Preferences.containsKey("intakeMotorKff")) {
      Preferences.initDouble("intakeMotorKff", Constants.Intake.kLauncherSubFF);
    }
    
    IntakeMotorMotor = new SparkMax(Constants.Intake.IntakeID, MotorType.kBrushless);
   // IntakeMotorMotor.restoreFactoryDefaults();

    IntakeMotorPID = IntakeMotorMotor.getClosedLoopController();
    config = new SparkMaxConfig();
    config
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    config.closedLoop.p(Constants.Intake.kLauncherSubP);
    config.closedLoop.i(Constants.Intake.kLauncherSubI);
    config.closedLoop.d(Constants.Intake.kLauncherSubD);
    config.closedLoop.pidf(Constants.Intake.kLauncherSubP, Constants.Intake.kLauncherSubI, 
    Constants.Intake.kLauncherSubD, Constants.Intake.kLauncherSubFF);
    config.closedLoop.outputRange(Constants.Intake.kLauncherSubMinOutput, Constants.Intake.kLauncherSubMaxOutput);
    IntakeMotorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    IntakeMotorEncoder = IntakeMotorMotor.getEncoder();

  //  IntakeMotorMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);


  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/
  public void updatePreferences() {
    if (Preferences.getDouble("intakeSpeed", intakeSpeed) != intakeSpeed) {
      intakeSpeed = Preferences.getDouble("intakeSpeed", intakeSpeed);
      ejectSpeed = Preferences.getDouble("ejectSpeed",ejectSpeed);
    }
    if (Preferences.getDouble("intakeMotorKp", Constants.Intake.kLauncherSubP) != Constants.Intake.kLauncherSubP || Preferences.getDouble("intakeMotorKi", Constants.Intake.kLauncherSubI) != Constants.Intake.kLauncherSubI || Preferences.getDouble("intakeMotorKd", Constants.Intake.kLauncherSubD) != Constants.Intake.kLauncherSubD || Preferences.getDouble("intakeMotorKff", Constants.Intake.kLauncherSubFF) != Constants.Intake.kLauncherSubFF) {
      intakeMotorKp = Preferences.getDouble("intakeMotorKp", Constants.Intake.kLauncherSubP);
      intakeMotorKi = Preferences.getDouble("intakeMotorKi", Constants.Intake.kLauncherSubI);
      intakeMotorKd = Preferences.getDouble("intakeMotorKd", Constants.Intake.kLauncherSubD);
      intakeMotorKff = Preferences.getDouble("intakeMotorKff", Constants.Intake.kLauncherSubFF);
      config.closedLoop.pidf(intakeMotorKp, intakeMotorKi, intakeMotorKd, intakeMotorKff);
      IntakeMotorMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
  }
  public void moveIntakeMotor(double rpm) {
    if (rpm < 0) {
      IntakeMotorMotor.setInverted(false);
      rpm = (-1 * rpm);
    }
    // else {
    //   IntakeMotorMotor.setInverted(false);
    // }
    
    double limitedSpeed = mSpeedLimiter.calculate(rpm);
    // this.config.
    IntakeMotorPID.setReference(limitedSpeed, ControlType.kVelocity);
    SmartDashboard.putNumber("IntakeMotorDesiredSpeed", limitedSpeed);
    SmartDashboard.putNumber("IntakeMotorVelocity", IntakeMotorEncoder.getVelocity());

  }

  public void stopIntakeMotorSubsystem() {
    //double limitedSpeed = mSpeedLimiter.calculate(0);
    IntakeMotorPID.setReference(0, ControlType.kVelocity);
  }


  public boolean hasCoral() {
    if (canRange.getIsDetected().getValueAsDouble() == 1 && canRange.getDistance().getValueAsDouble() < 0.1) {
      return true;
    }
    else {
      return false;
    }
  }

  /*---------------------------------- Custom public Functions ---------------------------------*/
  @Override
  public void periodic() {
    updatePreferences(); 

    SmartDashboard.putNumber("canrange is detected", canRange.getIsDetected().getValueAsDouble());
    SmartDashboard.putNumber("canRangeDistance", canRange.getDistance().getValueAsDouble());
  }
}


