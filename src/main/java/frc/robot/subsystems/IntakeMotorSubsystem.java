package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.intake;

public class IntakeMotorSubsystem extends SubsystemBase {

  /*-------------------------------- public instance variables ---------------------------------*/
  public CANSparkMax IntakeMotorMotor;

  public SparkPIDController IntakeMotorPID;

  public RelativeEncoder IntakeMotorEncoder;

  public SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1000);
  public double intakeSpeed = Constants.intake.ejectSpeed;

  public IntakeMotorSubsystem() {
    Preferences.initDouble("intakeSpeed", intakeSpeed);

    IntakeMotorMotor = new CANSparkMax(Constants.intake.IntakeID, MotorType.kBrushless);
   // IntakeMotorMotor.restoreFactoryDefaults();

    IntakeMotorPID = IntakeMotorMotor.getPIDController();
    IntakeMotorPID.setP(Constants.kLauncherSubP);
    IntakeMotorPID.setI(Constants.kLauncherSubI);
    IntakeMotorPID.setD(Constants.kLauncherSubD);
    IntakeMotorPID.setFF(Constants.kLauncherSubFF);
    IntakeMotorPID.setOutputRange(Constants.kLauncherSubMinOutput, Constants.kLauncherSubMaxOutput);

    IntakeMotorEncoder = IntakeMotorMotor.getEncoder();

  //  IntakeMotorMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    IntakeMotorMotor.setInverted(true);

  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/
  public double loadPreferences() {
    if (Preferences.getDouble("intakeSpeed", intakeSpeed) != intakeSpeed) {
      return Preferences.getDouble("intakeSpeed", intakeSpeed);
        } else {
           return intakeSpeed;
         } 
  }
  public void moveIntakeMotor(double rpm) {
    IntakeMotorMotor.setInverted(true);
    double limitedSpeed = mSpeedLimiter.calculate(rpm);
    IntakeMotorPID.setReference(limitedSpeed, ControlType.kVelocity);
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
}


