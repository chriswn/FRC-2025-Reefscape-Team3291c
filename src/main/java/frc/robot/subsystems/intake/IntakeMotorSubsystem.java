package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeMotorSubsystem extends SubsystemBase {

  public SparkMax IntakeMotorMotor;
  public PIDController PIDController;
  public SimpleMotorFeedforward simpleMotorFeedforward;
  public RelativeEncoder IntakeMotorEncoder;
  public SparkMaxConfig config;
  public CANrange canRange;

  public double intakeMotorKp = Preferences.getDouble("intakeMotorKp", Constants.Intake.intakeMotorKp);
  public double intakeMotorKi = Preferences.getDouble("intakeMotorKi", Constants.Intake.intakeMotorKi);
  public double intakeMotorKd = Preferences.getDouble("intakeMotorKd", Constants.Intake.intakeMotorKd);
  public double intakeMotorKv = Preferences.getDouble("intakeMotorKv", Constants.Intake.intakeMotorKv);
  public double intakeMotorKs = Preferences.getDouble("intakeMotorKs", Constants.Intake.intakeMotorKs);

  public double intakeSpeed = Preferences.getDouble("intakeSpeed", Constants.Intake.intakeSpeed);
  public double ejectSpeed = Preferences.getDouble("ejectSpeed", Constants.Intake.ejectSpeed);


  public IntakeMotorSubsystem() {
    this.simpleMotorFeedforward = new SimpleMotorFeedforward(intakeMotorKs, intakeMotorKv);
    this.PIDController= new PIDController(intakeMotorKp, intakeMotorKi, intakeMotorKd);

    this.canRange = new CANrange(Constants.Intake.CANrangeID);

    if (!Preferences.containsKey("ejectSpeed")) {
      Preferences.initDouble("ejectSpeed", ejectSpeed);
    }

    if (!Preferences.containsKey("intakeSpeed")) {
      Preferences.initDouble("intakeSpeed", intakeSpeed);
    }

    if (!Preferences.containsKey("intakeMotorKp")) {
      Preferences.initDouble("intakeMotorKp", Constants.Intake.intakeMotorKp);
    }
    
    if (!Preferences.containsKey("intakeMotorKi")) {
      Preferences.initDouble("intakeMotorKi", Constants.Intake.intakeMotorKi);
    }

    if (!Preferences.containsKey("intakeMotorKd")) {
      Preferences.initDouble("intakeMotorKd", Constants.Intake.intakeMotorKd);
    }

    if (!Preferences.containsKey("intakeMotorKs")) {
      Preferences.initDouble("intakeMotorKs", Constants.Intake.intakeMotorKs);
    }

    if (!Preferences.containsKey("intakeMotorKv")) {
      Preferences.initDouble("intakeMotorKv", Constants.Intake.intakeMotorKv);
    }

    
    this.IntakeMotorMotor = new SparkMax(Constants.Intake.IntakeID, MotorType.kBrushless);

    this.config = new SparkMaxConfig();
    this.config
        .inverted(Constants.Intake.reverseIntakeMotor)
        .idleMode(IdleMode.kBrake);
    this.IntakeMotorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.IntakeMotorEncoder = this.IntakeMotorMotor.getEncoder();
  }

  public void updatePreferences() {
    if (Preferences.getDouble("intakeSpeed", intakeSpeed) != intakeSpeed) {
      intakeSpeed = Preferences.getDouble("intakeSpeed", intakeSpeed);
    }
    if (Preferences.getDouble("ejectSpeed", ejectSpeed) != ejectSpeed) {
      ejectSpeed = Preferences.getDouble("ejectSpeed", ejectSpeed);
    }
    if (Preferences.getDouble("intakeMotorKp", Constants.Intake.intakeMotorKp) != Constants.Intake.intakeMotorKp || Preferences.getDouble("intakeMotorKi", Constants.Intake.intakeMotorKi) != Constants.Intake.intakeMotorKi || Preferences.getDouble("intakeMotorKd", Constants.Intake.intakeMotorKd) != Constants.Intake.intakeMotorKd) {
      intakeMotorKp = Preferences.getDouble("intakeMotorKp", Constants.Intake.intakeMotorKp);
      intakeMotorKi = Preferences.getDouble("intakeMotorKi", Constants.Intake.intakeMotorKi);
      intakeMotorKd = Preferences.getDouble("intakeMotorKd", Constants.Intake.intakeMotorKd);
       PIDController= new PIDController(intakeMotorKp, intakeMotorKi, intakeMotorKd);
    }
    if (Preferences.getDouble("intakeMotorKs", Constants.Intake.intakeMotorKs) != Constants.Intake.intakeMotorKs || Preferences.getDouble("intakeMotorKv", Constants.Intake.intakeMotorKv) != Constants.Intake.intakeMotorKv) {
      intakeMotorKs = Preferences.getDouble("intakeMotorKs", Constants.Intake.intakeMotorKs);
      intakeMotorKv = Preferences.getDouble("intakeMotorKv", Constants.Intake.intakeMotorKv);
      simpleMotorFeedforward = new SimpleMotorFeedforward(intakeMotorKs, intakeMotorKv);
    }
  }

  public void moveIntakeMotor(double rpm) {
    double power;
    power = PIDController.calculate(IntakeMotorEncoder.getVelocity(), rpm) + simpleMotorFeedforward.calculate(rpm);
    IntakeMotorMotor.setVoltage(power);

    SmartDashboard.putNumber("IntakeMotorVoltage", power);
    SmartDashboard.putNumber("IntakeMotorDesiredSpeed", rpm);
    SmartDashboard.putNumber("IntakeMotorVelocity", IntakeMotorEncoder.getVelocity());
  }

  public void stopIntakeMotorSubsystem() {
    IntakeMotorMotor.set(0);
  }

  public boolean hasCoral() {
    if (canRange.getIsDetected().getValueAsDouble() == 1 && canRange.getDistance().getValueAsDouble() < Constants.Intake.distanceSensorPointBlankRange) {
      return true;
    }
    else {
      return false;
    }
  }

  @Override
  public void periodic() {
    //updatePreferences(); //to be commented out

    SmartDashboard.putNumber("canrange is detected", canRange.getIsDetected().getValueAsDouble());
    SmartDashboard.putNumber("canRangeDistance", canRange.getDistance().getValueAsDouble());
  }
}


