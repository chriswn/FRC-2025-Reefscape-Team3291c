package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.config.CTREConfigs;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Swerve.Mod0;
import frc.lib.Math.SwerveOpt;
import frc.lib.config.*;

public class SwerveModule {
    // Module number identifier
    public int moduleNumber;

    // This is the last angle that we wanted to the wheel to move to.
    private Rotation2d lastAngle;

    /**
     * This is the starting angle of the wheel.   Since motors can be motor 
     * differently and the starting position of the shaft starts in a different
     * location, we need to know what the starting angle when the wheels are 
     * facing forward.   This is the basis of all angle changes after the 
     * robot is started up.
     */
    private Rotation2d angleOffset;

    // Used to angle the wheel in the directions we want to drive.
    private SparkMax angleMotor;

    // Used to rotate the wheel to move the robot.
    private SparkMax driveMotor;

    // Used to get the current position and state (velocity) of the drive motor
    private RelativeEncoder driveEncoder;

    private double angleMultiplier;
    /**
     * Currently defined, but not used in any calculations.   I'm assuming this was
     * first defined to be used for the angle motor, but we are doing that with the 
     * CANCoder (angleEncoder) defined below.
     */
    private RelativeEncoder integratedAngleEncoder;

    // Used to get the current angle (direction) of the wheel 
    private CANCoder angleEncoder;

    /** 
     * Used to PID control the angle motor power level.   The angle is converted from 
     * degrees to percentage of power.
     */
    private PIDController anglePid;
    private PIDController drivePid;

    //feedforward - def for auto maybe for teleop
    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Swerve.ffkS, Swerve.ffkV, Swerve.ffkA);


    /**
     * 
     * @param moduleNumber
     * @param moduleConstants
     */
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        // This is the module identifier.
        this.moduleNumber = moduleNumber;

        // This is the offset of the wheel in relation to the 0 position of the CANCoder.
        this.angleOffset  = moduleConstants.angleOffset;

        // Initializing the angle motor PID Controller with PID values
        this.anglePid = new PIDController(
            Swerve.angleKP,
            Swerve.angleKI,
            Swerve.angleKD
        );

        this.drivePid = new PIDController(
            Swerve.driveKP,
            Swerve.driveKI,
            Swerve.driveKD
        );

        //this.anglePid.enableContinuousInput(-Math.PI, Math.PI);

        this.angleMultiplier = moduleConstants.angleMultiplier;
        // Initializing the CANCoder with the desired device ID
        this.angleEncoder = new CANCoder(moduleConstants.canCoderId);//CANcoder(moduleConstants.canCoderId);
        configAngleEncoder();

        //configuring and initalizing drive motors and encoders
        this.driveMotor = new SparkMax(moduleConstants.driveMotorId, MotorType.kBrushless);
        this.driveEncoder = driveMotor.getEncoder();
        //this.driveController = driveMotor.getPIDController();
        this.configDriveMotor();

        //configuring and initalizing angle motors and encoders
        this.angleMotor = new SparkMax(moduleConstants.angleMotorId, MotorType.kBrushless);
        this.integratedAngleEncoder = angleMotor.getEncoder();
        // this.angleController = angleMotor.getPIDController();
        this.configAngleMotor();
    
        this.lastAngle = getState().angle;
    }

    /* 
     * 
     * This is intended to set the default strategy for each angle encoder (CANCoder)
     */
    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
        angleEncoder.configAllSettings(CTREConfigs.swerveCanCoderConfig);//Settings is null
    }

    /**
     * configDriveMotor() 
     * 
     * This is intended to set default values for this modules drive motor (SparkMAX)
     */
    private void configDriveMotor() {
        // Set motor to original specs
        //this.driveMotor.restoreFactoryDefaults();

        // This sets the transmission rate of data to all channels to be faster
        //**could cause error CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);

        // Sets the current limit to the drive motor in AMPs
       // this.driveMotor.setSmartCurrentLimit(Swerve.driveContinuousCurrentLimit);

        // Invert drive motor (currently, false)
        this.driveMotor.setInverted(Swerve.driveInvert);

        // // When the motor isn't being used, set to brake mode
        // this.driveMotor.setIdleMode(IdleMode.kBrake);

        // Set the conversion factor for velocity of the encoder. Multiplied by the native output units to give you velocity
        this.driveEncoder.setVelocityConversionFactor(Swerve.driveConversionVelocityFactor);

        // Set the conversion factor for position of the encoder. Multiplied by the native output units to give you position.
        this.driveEncoder.setPositionConversionFactor(Swerve.driveConversionPositionFactor);

        // Sets the voltage compensation setting for all modes on the SPARK and enables voltage compensation.
        this.driveMotor.enableVoltageCompensation(Swerve.voltageComp);

        // Writes values to the motor firmware
        this.driveMotor.burnFlash();

        // Set starting position of the drive encoder to 0.0
        this.driveEncoder.setPosition(0.0);
    }

    /**
     * 
     */
    private void configAngleMotor() {
        // Set motor to the original specs
       // this.angleMotor.restoreFactoryDefaults();

        // Sets the transmission rate of the position channel as faster.
        //**could cause error CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);

        CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);

        // Sets the current limit to the angle motor in AMPs
      //  this.angleMotor.setSmartCurrentLimit(Swerve.angleContinuousCurrentLimit);

        // Invert angle motor (currently, true)
        this.angleMotor.setInverted(Swerve.angleInvert);

        // WHen the motor isn't being used, set to brake mode
       // this.angleMotor.setIdleMode(IdleMode.kBrake);

        // Set the conversion factor for position of the encoder. Multiplied by the native output units to give you position.
        this.integratedAngleEncoder.setPositionConversionFactor(Swerve.angleConversionFactor);

        // Sets the voltage compensation setting for all modes on the SPARK and enables voltage compensation.
        this.angleMotor.enableVoltageCompensation(Swerve.voltageComp);

        // Writes values to the motor firmware
        this.angleMotor.burnFlash();
    }

    /**
     * 
     * @param desiredState
     */
    //stolen from crimson robotics #2526
    private boolean setAngle(SwerveModuleState desiredState) {
        boolean invertDriveMotor = false;
        // Get the current state of the swerve module (i.e. What direction is it facing?)
        SwerveModuleState currentState = this.getState();

        // What is the current direction (in degrees) the swerve module is facing?
        double currentDegrees = (currentState.angle.getDegrees());
        
        // We want the current degrees between [-180, 180]. When looking at a circle
        // -190 is equivalent to 170 so the below ternaries convert numbers less than -180
        // or greater than 180 into [-180,180] 
        currentDegrees = currentDegrees < -180 ? currentDegrees + 360 : currentDegrees;
        currentDegrees = currentDegrees > 180 ? currentDegrees - 360 : currentDegrees;

        // Next figure out the what direction the service module should be pointing. We mode the number
        // (% 360) to make sure the number is between [-360,360]
        double desiredDegrees = desiredState.angle.getDegrees() % 360;
        // The following logic calculates the the different between the desired angle of the
        // swerve module to the desired angle. The final result will be some number
        // in the range [-180, 180]
        double diff = (currentDegrees - desiredDegrees + 180) % 360 - 180;
        diff = diff < -180 ? diff + 360 : diff;
        diff = diff > 180 ? diff - 360 : diff;

        // In theory, we could turn the swerve to this "diff" angle and everything would work but its
        // possible to be more efficient. For example, if the operator pushes the joystick forward,
        // then pulls it full back, the code above will turn the swerve module 180 degrees when
        // instead we could just reverse the direction of the drive motor.

        // If the `diff` is greater than 90 or less than -90 then invert the drive motor
        // and change the diff.
        // -120 is equivalent to 60 with an inverted drive direction.
        // 120 is equivalent to -60 with an inverted drive direction.
        if (Math.abs(diff) > 90) {
            invertDriveMotor = true;
            if (diff < 0) {
                diff += 180;
            } else {
                diff -= 180;
            }
        }

        // If we are a degree or less off, the turning motor will not do anything, else, calculate the pid
        // values based off the diff and 0.
        double turningMotorValue = Math.abs(diff) < 1 ? 0 : anglePid.calculate(diff, 0);
        // Motors can only take a value in the range [-1,1] but the PID may output a value outside
        // this range, thus we "clamp" the output of the PID such that -1.5 would just be -1.
        turningMotorValue = turningMotorValue > 1 ? 1 : turningMotorValue;
        turningMotorValue = turningMotorValue < -1 ? -1 : turningMotorValue;

        // Turn the motor to the desired position
        angleMotor.set(turningMotorValue);

        // Return the direction the drive motor is expected to go.
        return invertDriveMotor;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop, boolean invertDriveMotor) {
        // If we are in open loop mode, set the drive motor to the desired speed
        // if (isOpenLoop) {
            double driveValue = (desiredState.speedMetersPerSecond / Swerve.maxSpeed);
            driveValue = drivePid.calculate(driveEncoder.getVelocity(), desiredState.speedMetersPerSecond);
            //driveValue = feedForward.calculate(desiredState.speedMetersPerSecond/2) + drivePid.calculate(driveEncoder.getVelocity(), desiredState.speedMetersPerSecond/2);
            driveValue = invertDriveMotor ? driveValue * -1 : driveValue;
            driveMotor.set(driveValue);
        // } else {
        //     // If we are in closed loop mode, set the drive motor to the desired speed
        //     this.driveMotor.set(
        //         this.driveEncoder.getVelocityConversionFactor() * 
        //        desiredState.speedMetersPerSecond / Swerve.maxSpeed
        //    );
        // }
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition()  - this.angleOffset.getDegrees());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(this.driveEncoder.getVelocity(), this.getCanCoder());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            (this.driveEncoder.getPosition()), // * (Swerve.wheelCircumference / (Swerve.driveGearRatio * 42)// Actually forgot where I got this from lmao
            this.getCanCoder()
        );
    }
    
    public void resetToAbsolute() {
      this.lastAngle = Rotation2d.fromDegrees(0);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        //desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        boolean invertDriveMotor = setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop, invertDriveMotor);
    }
}