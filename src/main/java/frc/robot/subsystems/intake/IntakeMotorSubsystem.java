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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Elevator.PID;

public class IntakeMotorSubsystem extends SubsystemBase {

  /*-------------------------------- public instance variables ---------------------------------*/
  public SparkMax IntakeMotorMotor;

  public PIDController PIDController;
  public TrapezoidProfile.Constraints tConstraints;
  public SimpleMotorFeedforward simpleMotorFeedforward;

  public TrapezoidProfile.State goal;

  public RelativeEncoder IntakeMotorEncoder;

  //public SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1000);
  public SparkMaxConfig config;

  public CANrange canRange;

  public double intakeMotorKp = Preferences.getDouble("intakeMotorKp", Constants.Intake.kLauncherSubP);
  public double intakeMotorKi = Preferences.getDouble("intakeMotorKi", Constants.Intake.kLauncherSubI);
  public double intakeMotorKd = Preferences.getDouble("intakeMotorKd", Constants.Intake.kLauncherSubD);
  public double intakeMotorKv = Preferences.getDouble("intakeMotorKv", Constants.Intake.kLauncherSubV);
  public double intakeMotorKs = Preferences.getDouble("intakeMotorKs", Constants.Intake.kLauncherSubS);

  public double intakeMotorMaxAcceleration = Preferences.getDouble("intakeMotorMaxAcceleration", Constants.Intake.intakeMotorMaxAcceleration);
  public double intakeMotorMaxVelocity = Preferences.getDouble("intakeMotorMaxVelocity", Constants.Intake.intakeMotorMaxVelocity);

  public double intakeSpeed = Preferences.getDouble("intakeSpeed", Constants.Intake.intakeSpeed);
  public double ejectSpeed = Preferences.getDouble("ejectSpeed", Constants.Intake.ejectSpeed);




  public IntakeMotorSubsystem() {
    this.simpleMotorFeedforward = new SimpleMotorFeedforward(intakeMotorKs, intakeMotorKv);
    this.tConstraints = new TrapezoidProfile.Constraints(intakeMotorMaxVelocity, intakeMotorMaxAcceleration);
    this.PIDController= new PIDController(intakeMotorKp, intakeMotorKi, intakeMotorKd);

    this.canRange = new CANrange(Constants.Intake.CANrangeID);

    if (!Preferences.containsKey("ejectSpeed")) {
      Preferences.initDouble("ejectSpeed", ejectSpeed);
    }
    if (!Preferences.containsKey("intakeSpeed")) {
      Preferences.initDouble("intakeSpeed", intakeSpeed);
    }
    if (!Preferences.containsKey("intakeMotorMaxAcceleration")) {
      Preferences.initDouble("intakeMotorMaxAcceleration", Constants.Intake.intakeMotorMaxAcceleration);
    }
    if (!Preferences.containsKey("intakeMotorMaxVelocity")) {
      Preferences.initDouble("intakeMotorMaxVelocity", Constants.Intake.intakeMotorMaxVelocity);
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
    if (!Preferences.containsKey("intakeMotorKs")) {
      Preferences.initDouble("intakeMotorKs", Constants.Intake.kLauncherSubS);
    }
    if (!Preferences.containsKey("intakeMotorKv")) {
      Preferences.initDouble("intakeMotorKv", Constants.Intake.kLauncherSubV);
    }
    
    this.IntakeMotorMotor = new SparkMax(Constants.Intake.IntakeID, MotorType.kBrushless);
   // IntakeMotorMotor.restoreFactoryDefaults();

    this.config = new SparkMaxConfig();
    this.config
        .inverted(Constants.Intake.reverseIntakeMotor)
        .idleMode(IdleMode.kBrake);
    this.IntakeMotorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.IntakeMotorEncoder = this.IntakeMotorMotor.getEncoder();

  //  IntakeMotorMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);


  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/
  public void updatePreferences() {
    if (Preferences.getDouble("intakeSpeed", intakeSpeed) != intakeSpeed) {
      intakeSpeed = Preferences.getDouble("intakeSpeed", intakeSpeed);
    }
    if (Preferences.getDouble("ejectSpeed", ejectSpeed) != ejectSpeed) {
      ejectSpeed = Preferences.getDouble("ejectSpeed", ejectSpeed);
    }
    // if (Preferences.getDouble("intakeMotorMaxAcceleration", Constants.Intake.intakeMotorMaxAcceleration) != intakeMotorMaxAcceleration) {
    //   intakeMotorMaxAcceleration = Preferences.getDouble("intakeMotorMaxAcceleration", Constants.Intake.intakeMotorMaxAcceleration);
    //    PIDController= new ProfiledPIDController(intakeMotorKp, intakeMotorKi, intakeMotorKd, new TrapezoidProfile.Constraints(intakeMotorMaxVelocity, intakeMotorMaxAcceleration));

    // }
    // if (Preferences.getDouble("intakeMotorMaxVelocity", Constants.Intake.intakeMotorMaxVelocity) != intakeMotorMaxVelocity) {
    //   intakeMotorMaxVelocity = Preferences.getDouble("intakeMotorMaxVelocity", Constants.Intake.intakeMotorMaxVelocity);
    //    PIDController= new ProfiledPIDController(intakeMotorKp, intakeMotorKi, intakeMotorKd, new TrapezoidProfile.Constraints(intakeMotorMaxVelocity, intakeMotorMaxAcceleration));
    // }
    if (Preferences.getDouble("intakeMotorKp", Constants.Intake.kLauncherSubP) != Constants.Intake.kLauncherSubP || Preferences.getDouble("intakeMotorKi", Constants.Intake.kLauncherSubI) != Constants.Intake.kLauncherSubI || Preferences.getDouble("intakeMotorKd", Constants.Intake.kLauncherSubD) != Constants.Intake.kLauncherSubD) {
      intakeMotorKp = Preferences.getDouble("intakeMotorKp", Constants.Intake.kLauncherSubP);
      intakeMotorKi = Preferences.getDouble("intakeMotorKi", Constants.Intake.kLauncherSubI);
      intakeMotorKd = Preferences.getDouble("intakeMotorKd", Constants.Intake.kLauncherSubD);
       PIDController= new PIDController(intakeMotorKp, intakeMotorKi, intakeMotorKd);
    }
    if (Preferences.getDouble("intakeMotorKs", Constants.Intake.kLauncherSubS) != Constants.Intake.kLauncherSubS || Preferences.getDouble("intakeMotorKv", Constants.Intake.kLauncherSubV) != Constants.Intake.kLauncherSubV) {
      intakeMotorKs = Preferences.getDouble("intakeMotorKs", Constants.Intake.kLauncherSubS);
      intakeMotorKv = Preferences.getDouble("intakeMotorKv", Constants.Intake.kLauncherSubV);
      simpleMotorFeedforward = new SimpleMotorFeedforward(intakeMotorKs, intakeMotorKv);
    }
  }

  public void moveIntakeMotor(double rpm, boolean isInversed) {
    double isNegative;
    if (isInversed) {
      isNegative = -1.0;
    }
    else {
      isNegative = 1.0;
    }
    
    double power;
    power = PIDController.calculate(IntakeMotorEncoder.getVelocity(), rpm) + simpleMotorFeedforward.calculate(rpm);
    power *= isNegative;
    IntakeMotorMotor.setVoltage(power);
    SmartDashboard.putNumber("IntakeMotorVoltage", power);

    SmartDashboard.putNumber("IntakeMotorDesiredSpeed", rpm);
    SmartDashboard.putNumber("IntakeMotorVelocity", IntakeMotorEncoder.getVelocity());

  }

  public void stopIntakeMotorSubsystem() {
    //double limitedSpeed = mSpeedLimiter.calculate(0);
    IntakeMotorMotor.set(0);
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


