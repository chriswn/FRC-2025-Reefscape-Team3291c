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

    // Tunable parameters (Add these)
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

    
    // State
    private Pose2d targetPose = new Pose2d();
    private boolean hasValidTarget;

    public AutoAlignCommand(VisionSim visionSim, SwerveSubsystem drivebase, 
                          int targetTagId, Pose2d targetOffset) {
        this.visionSim = visionSim;
        this.drivebase = drivebase;
        this.targetTagId = targetTagId;
        this.targetOffset = targetOffset;

        // Tolerances (looser for simulation)
        this.positionTolerance = RobotBase.isSimulation() ? 0.1 : 0.03;
        this.rotationTolerance = RobotBase.isSimulation() ? 3.0 : 1.5;
        this.maxLinearSpeed = RobotBase.isSimulation() ? 2.0 : 4.5;
        this.maxAngularSpeed = RobotBase.isSimulation() ? Math.PI : 2 * Math.PI;

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
        xController.reset();
        yController.reset();
        thetaController.reset();
        hasValidTarget = false;
        
        // Get target pose from field layout
        Constants.Vision.APRILTAG_FIELD_LAYOUT.getTagPose(targetTagId).ifPresent(tagPose -> {
            Transform2d offsetTransform = new Transform2d(
                targetOffset.getTranslation(),  // Translation2d from Pose2d
                targetOffset.getRotation()      // Rotation2d from Pose2d
            );
            

            targetPose = tagPose.toPose2d().transformBy(offsetTransform);
            hasValidTarget = true;
        });
        
        if (!hasValidTarget) {
            DriverStation.reportWarning("AprilTag " + targetTagId + " not found in field layout!", false);
        }
    }
    

    @Override
    public void execute() {
        if (!hasValidTarget) return;

        xController.setP(pGainX.get());
        xController.setD(dGainX.get());
        yController.setP(pGainY.get());
        yController.setD(dGainY.get());
        thetaController.setP(pGainTheta.get());
        thetaController.setD(dGainTheta.get());

        // Use fused pose from drivebase (odometry + vision)
        Pose2d currentPose = drivebase.getPose();
        
        // Calculate target-relative position
        Pose2d relativePose = currentPose.relativeTo(targetPose);
        
        // Calculate PID outputs with constraints
        double xSpeed = -clamp(xController.calculate(relativePose.getX(), 0), maxLinearSpeed);
        double ySpeed = -clamp(yController.calculate(relativePose.getY(), 0), maxLinearSpeed);
        double thetaSpeed = -clamp(thetaController.calculate(
            currentPose.getRotation().getDegrees(), 
            targetPose.getRotation().getDegrees()
        ), maxAngularSpeed);

        // Create field-relative speeds
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, thetaSpeed, drivebase.getHeading()
        );
        // Apply smoothing to the speeds
        double smoothing = speedSmoothing.get();
        ChassisSpeeds smoothedSpeeds = new ChassisSpeeds(
            prevSpeeds.vxMetersPerSecond * smoothing + speeds.vxMetersPerSecond * (1 - smoothing),
            prevSpeeds.vyMetersPerSecond * smoothing + speeds.vyMetersPerSecond * (1 - smoothing),
            prevSpeeds.omegaRadiansPerSecond * smoothing + speeds.omegaRadiansPerSecond * (1 - smoothing)
        );
        
        // Use tunable deadband
        if (Math.abs(relativePose.getX()) < deadband.get()) xSpeed = 0;
        if (Math.abs(relativePose.getY()) < deadband.get()) ySpeed = 0;
    
    prevSpeeds = smoothedSpeeds;
    drivebase.drive(smoothedSpeeds);

        drivebase.drive(speeds);
        updateTelemetry(relativePose, speeds);
    }

    private double clamp(double value, double max) {
          if (Math.abs(value) < 0.1) return 0;
            return Math.copySign(Math.min(Math.abs(value), max), value);
            
        }
    

    private void updateTelemetry(Pose2d relativePose, ChassisSpeeds speeds) {
        SmartDashboard.putNumber("AutoAlign/X Error", relativePose.getX());
        SmartDashboard.putNumber("AutoAlign/Y Error", relativePose.getY());
        SmartDashboard.putNumber("AutoAlign/Theta Error", relativePose.getRotation().getDegrees());
        SmartDashboard.putNumber("AutoAlign/X Speed", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("AutoAlign/Y Speed", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("AutoAlign/Theta Speed", Math.toDegrees(speeds.omegaRadiansPerSecond));
        SmartDashboard.putNumberArray("AutoAlign/Target Pose", new double[]{
        targetPose.getX(),
        targetPose.getY(),
        targetPose.getRotation().getDegrees()
    });
    Pose2d currentPose = drivebase.getPose();
    SmartDashboard.putBoolean("AutoAlign/Active", hasValidTarget);
    SmartDashboard.putNumberArray("AutoAlign/Current Pose", 
    new double[]{
        currentPose.getX(),
        currentPose.getY(),
        currentPose.getRotation().getDegrees()
    });
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.drive(new ChassisSpeeds());
        SmartDashboard.putBoolean("AutoAlign/Active", false);
    }

    @Override
    public boolean isFinished() {
        return !hasValidTarget || 
               (xController.atSetpoint() && 
                yController.atSetpoint() && 
                thetaController.atSetpoint());
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}