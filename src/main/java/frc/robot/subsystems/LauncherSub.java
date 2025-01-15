package frc.robot.subsystems;

import java.util.function.Consumer;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.MutableMeasure.mutable;



public class LauncherSub extends SubsystemBase {

  /*-------------------------------- public instance variables ---------------------------------*/
  //public static LauncherSub mInstance;
  //public PeriodicIO mPeriodicIO;

 // public static LauncherSub getInstance() {
 //   if (mInstance == null) {
   //   mInstance = new LauncherSub();
    //}
    //return mInstance;
  //}

  public CANSparkMax mLeftLauncherSubMotor;
  public CANSparkMax mRightLauncherSubMotor;

  public SparkPIDController mLeftLauncherSubPID;
  public SparkPIDController mRightLauncherSubPID;

  public RelativeEncoder mLeftLauncherSubEncoder;
  public RelativeEncoder mRightLauncherSubEncoder;

  public SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1000);
  public SysIdRoutine routine;
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity = mutable(MetersPerSecond.of(0));
   private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Voltage volts) -> {
                mLeftLauncherSubMotor.setVoltage(volts.in(Volts));
                mRightLauncherSubMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            mLeftLauncherSubMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(mLeftLauncherSubEncoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(mLeftLauncherSubEncoder.getCountsPerRevolution(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            mRightLauncherSubMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(mRightLauncherSubEncoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(mRightLauncherSubEncoder.getCountsPerRevolution(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

  public LauncherSub() {
    // super("LauncherSub");

  //  mPeriodicIO = new PeriodicIO();

    mLeftLauncherSubMotor = new CANSparkMax(Constants.kLauncherSubLeftMotorId, MotorType.kBrushless);
    mRightLauncherSubMotor = new CANSparkMax(Constants.kLauncherSubRightMotorId, MotorType.kBrushless);
   // mLeftLauncherSubMotor.restoreFactoryDefaults();
   //mRightLauncherSubMotor.restoreFactoryDefaults();

    mLeftLauncherSubPID = mLeftLauncherSubMotor.getPIDController();
    mLeftLauncherSubPID.setP(Constants.kLauncherSubP);
    mLeftLauncherSubPID.setI(Constants.kLauncherSubI);
    mLeftLauncherSubPID.setD(Constants.kLauncherSubD);
    mLeftLauncherSubPID.setFF(Constants.kLauncherSubFF);
    mLeftLauncherSubPID.setOutputRange(Constants.kLauncherSubMinOutput, Constants.kLauncherSubMaxOutput);

    mRightLauncherSubPID = mRightLauncherSubMotor.getPIDController();
    mRightLauncherSubPID.setP(Constants.kLauncherSubP);
    mRightLauncherSubPID.setI(Constants.kLauncherSubI);
    mRightLauncherSubPID.setD(Constants.kLauncherSubD);
    mRightLauncherSubPID.setFF(Constants.kLauncherSubFF);
    mRightLauncherSubPID.setOutputRange(Constants.kLauncherSubMinOutput, Constants.kLauncherSubMaxOutput);

    mLeftLauncherSubEncoder = mLeftLauncherSubMotor.getEncoder();
    mRightLauncherSubEncoder = mRightLauncherSubMotor.getEncoder();

    mLeftLauncherSubMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mRightLauncherSubMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    mLeftLauncherSubMotor.setInverted(true);
    mRightLauncherSubMotor.setInverted(false);
    

    
  // Create a new SysId routine for characterizing the drive.
 
  }

  

//  public static class PeriodicIO {
//    double LauncherSub_rpm = 0.0;
//  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  public void stop() {
    stopLauncherSub();
  }

  // public void outputTelemetry() {
  //   putNumber("Speed (RPM):", LauncherSub_rpm);
  //   putNumber("Left speed:", mLeftLauncherSubEncoder.getVelocity());
  //   putNumber("Right speed:", mRightLauncherSubEncoder.getVelocity());
  // }



  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public void setSpeed(double rpm) {
    mLeftLauncherSubMotor.setInverted(true);
    mRightLauncherSubMotor.setInverted(false);
    double limitedSpeed = mSpeedLimiter.calculate(rpm);
    mLeftLauncherSubPID.setReference(limitedSpeed, ControlType.kVelocity);
    mRightLauncherSubPID.setReference(limitedSpeed, ControlType.kVelocity);
  }
  public void setSpeedOpposite(double rpm) {
    mLeftLauncherSubMotor.setInverted(false);
    mRightLauncherSubMotor.setInverted(true);
    double limitedSpeed = mSpeedLimiter.calculate(rpm);
    mLeftLauncherSubPID.setReference(limitedSpeed, ControlType.kVelocity);
    mRightLauncherSubPID.setReference(limitedSpeed, ControlType.kVelocity);
  }

  public void stopLauncherSub() {
    //double limitedSpeed = mSpeedLimiter.calculate(0);
    mLeftLauncherSubPID.setReference(0, ControlType.kVelocity);
    mRightLauncherSubPID.setReference(0, ControlType.kVelocity);
  }

  /*---------------------------------- Custom public Functions ---------------------------------*/
}


