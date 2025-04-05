package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.VisionSim;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.photonvision.EstimatedRobotPose;
import java.util.Optional;

public class AutoAlignCommand extends Command {
    
    private final VisionSim visionSim;
    private final SwerveSubsystem drivebase;
    private final PIDController xController = new PIDController(0.3, 0, 0.2);
    private final PIDController yController = new PIDController(0.3, 0, 0.2);
    private final PIDController thetaController = new PIDController(0.05, 0, 0.05);

    // Tunable parameters
    private final LoggedNetworkNumber pGainX = new LoggedNetworkNumber("/Tuning/PID/kP_X", 0.3);
    private final LoggedNetworkNumber pGainY = new LoggedNetworkNumber("/Tuning/PID/kP_Y", 0.3);
    private final LoggedNetworkNumber pGainTheta = new LoggedNetworkNumber("/Tuning/PID/kP_Theta", 0.05);
    private final LoggedNetworkNumber dGainX = new LoggedNetworkNumber("/Tuning/PID/kD_X", 0.2);
    private final LoggedNetworkNumber dGainY = new LoggedNetworkNumber("/Tuning/PID/kD_Y", 0.2);
    private final LoggedNetworkNumber dGainTheta = new LoggedNetworkNumber("/Tuning/PID/kD_Theta", 0.05);
    private final LoggedNetworkNumber speedSmoothing = new LoggedNetworkNumber("/Tuning/Smoothing", 0.5);
    private final LoggedNetworkNumber deadband = new LoggedNetworkNumber("/Tuning/Deadband", 0.1);
    
    // Configuration
    private final int targetTagId;
    private final Pose2d targetOffset;
    private final double positionTolerance;
    private final double rotationTolerance;
    private final double maxLinearSpeed;
    private final double maxAngularSpeed;
    private ChassisSpeeds prevSpeeds = new ChassisSpeeds();
    private double startTime;

    // Minimum runtime before allowing exit
    private static final double MIN_RUN_TIME = 1.0; // Adjust the value as needed

    // State
    private Pose2d targetPose = new Pose2d();
    private boolean hasValidTarget;
    private boolean isAtSetpoint;

    public AutoAlignCommand(VisionSim visionSim, SwerveSubsystem drivebase, 
                          int targetTagId, Pose2d targetOffset) {
        this.visionSim = visionSim;
        this.drivebase = drivebase;
        this.targetTagId = targetTagId;
        this.targetOffset = targetOffset;

        this.positionTolerance = RobotBase.isSimulation() ? 0.1 : 0.03;
        this.rotationTolerance = RobotBase.isSimulation() ? 3.0 : 1.5;
        this.maxLinearSpeed = RobotBase.isSimulation() ? 2.0 : 4.5;
        this.maxAngularSpeed = RobotBase.isSimulation() ? Math.PI : 3 * Math.PI; // Increased rotation speed

        configureControllers();
        addRequirements(drivebase);
    }

    private void configureControllers() {
        thetaController.enableContinuousInput(-180, 180);
        xController.setTolerance(positionTolerance);
        yController.setTolerance(positionTolerance);
        thetaController.setTolerance(rotationTolerance);

        xController.setIntegratorRange(-0.5, 0.5);
        yController.setIntegratorRange(-0.5, 0.5);
        thetaController.setIntegratorRange(-0.5, 0.5);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        isAtSetpoint = false;
        xController.reset();
        yController.reset();
        thetaController.reset();
        hasValidTarget = false;
        prevSpeeds = new ChassisSpeeds(); // Reset smoothing buffer

        Constants.Vision.APRILTAG_FIELD_LAYOUT.getTagPose(targetTagId).ifPresent(tagPose -> {
            Transform2d offsetTransform = new Transform2d(
                targetOffset.getTranslation(),
                targetOffset.getRotation()
            );
            targetPose = tagPose.toPose2d().transformBy(offsetTransform);
            hasValidTarget = true;
        });
        
        if (!hasValidTarget) {
            DriverStation.reportWarning("AprilTag " + targetTagId + " not found!", false);
        }
    }

    @Override
    public void execute() {
        if (!hasValidTarget) return;

        // Update controller gains
        xController.setP(pGainX.get());
        xController.setD(dGainX.get());
        yController.setP(pGainY.get());
        yController.setD(dGainY.get());
        thetaController.setP(pGainTheta.get());
        thetaController.setD(dGainTheta.get());

        Pose2d currentPose = drivebase.getPose();
        Pose2d relativePose = currentPose.relativeTo(targetPose);

        // Calculate rotation error with proper wrapping
        double thetaError = currentPose.getRotation().minus(targetPose.getRotation()).getDegrees();
        thetaError = (thetaError + 180) % 360 - 180; // Force wrap to [-180, 180)

        // Apply deadband before PID calculation
        double xError = Math.abs(relativePose.getX()) > deadband.get() ? relativePose.getX() : 0;
        double yError = Math.abs(relativePose.getY()) > deadband.get() ? relativePose.getY() : 0;

        // Calculate PID outputs
        double xSpeed = -clamp(xController.calculate(xError, 0), maxLinearSpeed);
        double ySpeed = -clamp(yController.calculate(yError, 0), maxLinearSpeed);
        double thetaSpeed = -clamp(thetaController.calculate(thetaError, 0), maxAngularSpeed);

        // Generate and smooth speeds
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, thetaSpeed, drivebase.getHeading()
        );
        
        // double smoothing = speedSmoothing.get();
        ChassisSpeeds smoothedSpeeds = new ChassisSpeeds();
        //     prevSpeeds.vxMetersPerSecond * smoothing + speeds.vxMetersPerSecond * (1 - smoothing),
        //     prevSpeeds.vyMetersPerSecond * smoothing + speeds.vyMetersPerSecond * (1 - smoothing),
        //     prevSpeeds.omegaRadiansPerSecond * smoothing + speeds.omegaRadiansPerSecond * (1 - smoothing)
        // );

        if (Timer.getFPGATimestamp() - startTime < 0.3) { // First 300ms
            // Use heavier smoothing during initial alignment
            smoothedSpeeds = new ChassisSpeeds(
                prevSpeeds.vxMetersPerSecond * 0.8 + speeds.vxMetersPerSecond * 0.2,
                prevSpeeds.vyMetersPerSecond * 0.8 + speeds.vyMetersPerSecond * 0.2,
                prevSpeeds.omegaRadiansPerSecond * 0.8 + speeds.omegaRadiansPerSecond * 0.2
            );
        } else {
            // Use normal smoothing
            double smoothing = speedSmoothing.get();
            smoothedSpeeds = new ChassisSpeeds(
                prevSpeeds.vxMetersPerSecond * smoothing + speeds.vxMetersPerSecond * (1 - smoothing),
                prevSpeeds.vyMetersPerSecond * smoothing + speeds.vyMetersPerSecond * (1 - smoothing),
                prevSpeeds.omegaRadiansPerSecond * smoothing + speeds.omegaRadiansPerSecond * (1 - smoothing)
            );
        }
        
        prevSpeeds = smoothedSpeeds;
        drivebase.drive(smoothedSpeeds); // Single drive command

        updateTelemetry(relativePose, smoothedSpeeds, thetaError);
    }

    private double clamp(double value, double max) {
        return Math.copySign(Math.min(Math.abs(value), max), value);
    }

    private void updateTelemetry(Pose2d relativePose, ChassisSpeeds speeds, double thetaError) {
        // Current state
        Pose2d currentPose = drivebase.getPose();
        SmartDashboard.putNumberArray("AutoAlign/Current Pose", 
            new double[]{currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees()});
        
        // Target state
        SmartDashboard.putNumberArray("AutoAlign/Target Pose", 
            new double[]{targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()});
        
        // Errors
        SmartDashboard.putNumber("AutoAlign/X Error (m)", relativePose.getX());
        SmartDashboard.putNumber("AutoAlign/Y Error (m)", relativePose.getY());
        SmartDashboard.putNumber("AutoAlign/Theta Error (deg)", thetaError);
        
        // Speeds
        SmartDashboard.putNumber("AutoAlign/X Speed (m/s)", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("AutoAlign/Y Speed (m/s)", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("AutoAlign/Theta Speed (deg/s)", 
            Math.toDegrees(speeds.omegaRadiansPerSecond));
        
        // Tunable parameters
        SmartDashboard.putNumber("AutoAlign/Current kP_Theta", pGainTheta.get());
        SmartDashboard.putNumber("AutoAlign/Current kD_Theta", dGainTheta.get());
            // Add alignment progress
            SmartDashboard.putNumber("AutoAlign/Progress", 
                Math.min(1.0, (Timer.getFPGATimestamp() - startTime) / MIN_RUN_TIME));
            
            // Add stability indicators
            SmartDashboard.putBoolean("AutoAlign/Stable", 
                (Timer.getFPGATimestamp() - startTime) > 0.5);
        
    }

    // @Override
    // public void end(boolean interrupted) {
    //     drivebase.drive(new ChassisSpeeds());
    //     SmartDashboard.putBoolean("AutoAlign/Active", false);
    // }

    @Override
    public boolean isFinished() {
         boolean atSetpoint = xController.atSetpoint() && 
                            yController.atSetpoint() && 
                            thetaController.atSetpoint();
        
        // Require minimum runtime before allowing exit
        boolean minTimeElapsed = (Timer.getFPGATimestamp() - startTime) > MIN_RUN_TIME;
        isAtSetpoint = atSetpoint && minTimeElapsed;
        
        return !hasValidTarget || isAtSetpoint;

        // return !hasValidTarget || 
        //        (xController.atSetpoint() && 
        //         yController.atSetpoint() && 
        //         thetaController.atSetpoint());
         
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Smooth stop instead of immediate zero
        ChassisSpeeds stopSpeeds = new ChassisSpeeds(
            prevSpeeds.vxMetersPerSecond * 0.5,
            prevSpeeds.vyMetersPerSecond * 0.5,
            prevSpeeds.omegaRadiansPerSecond * 0.5
        );
        drivebase.drive(stopSpeeds);
        
        new Thread(() -> {
            try { Thread.sleep(150); } // Allow 150ms for slowdown
            catch (InterruptedException e) {}
            drivebase.drive(new ChassisSpeeds());
        }).start();
        
        SmartDashboard.putBoolean("AutoAlign/Active", false);
    }
}