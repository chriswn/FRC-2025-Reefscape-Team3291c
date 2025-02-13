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
  public double IntakeSpeed = Constants.Intake.ejectSpeed;
  public SparkMaxConfig config;

  public CANrange canRange;

  public IntakeMotorSubsystem() {
    canRange = new CANrange(0);
    Preferences.initDouble("IntakeSpeed", IntakeSpeed);

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
  public void moveIntakeMotor(double rpm) {
    IntakeMotorMotor.setInverted(true);
    double limitedSpeed = mSpeedLimiter.calculate(rpm);
    // this.config.
    this.IntakeMotorPID.setReference(limitedSpeed, ControlType.kVelocity);
  }
  public void moveIntakeMotorReversed(double rpm) {
    IntakeMotorMotor.setInverted(false);
    double limitedSpeed = mSpeedLimiter.calculate(rpm);
    IntakeMotorPID.setReference(limitedSpeed, ControlType.kVelocity);
  }

  public void stopIntakeMotorSubsystem() {
    //double limitedSpeed = mSpeedLimiter.calculate(0);
    IntakeMotorPID.setReference(0, ControlType.kVelocity);
  }

  /*---------------------------------- Custom public Functions ---------------------------------*/
  @Override
  public void periodic() {
    StatusSignal distance = canRange.getDistance();    
    SmartDashboard.putNumber("canRangeDistance", distance.getValueAsDouble());
  }
}


