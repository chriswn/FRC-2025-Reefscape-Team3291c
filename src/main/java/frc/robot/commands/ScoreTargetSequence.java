package frc.robot.commands;

import java.util.Arrays;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeMotorSubsystem;
import frc.robot.subsystems.ScoringTargetManager;
import frc.robot.subsystems.ScoringTarget;
import frc.robot.commands.ElevatorCMDs.SetElevatorLevelCommand;
import frc.robot.commands.IntakeMotorCMDs.ESpitCMD;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.Constants;

/**
 * Full sequence to call next scoring target, align to its AprilTag,
 * move elevator, and score the coral.
 */
public class ScoreTargetSequence extends SequentialCommandGroup {

    private final ScoringTarget target;
   
   
    private void logInitialization() {
        System.out.println("[ScoreTargetSequence] Initialized");

        SmartDashboard.putString("ScoreTargetSequence/Status", "Initialized");
        SmartDashboard.putString("ScoreTargetSequence/Target",
            "Face: " + target.getFace() +
            ", Level: " + target.getLevel() +
            ", TagID: " + target.getTagId());
    }


    public ScoreTargetSequence(
        VisionSubsystem vision,
        SwerveSubsystem drivebase,
        ElevatorSubsystem elevator,
        IntakeMotorSubsystem intakeMotor,
        ScoringTargetManager manager
    ) {
        
        addRequirements(vision, drivebase, elevator, intakeMotor);

        if (manager == null) {
            System.err.println("[ERROR] ScoringTargetManager is not initialized!");
            throw new IllegalStateException("ScoringTargetManager is not initialized.");
        }

        // Pick current or next target
        target = manager.getCurrentTarget().orElse(manager.getNextTarget());
        System.out.println("Scoring Target Called: " + target);

        if (target == null) {
            System.err.println("[ERROR] No scoring targets available!");
            System.err.println("Current target: " + manager.getCurrentTarget());
            System.err.println("Next target: " + manager.getNextTarget());
            throw new IllegalStateException("No target available for scoring.");
        }

        // Save the target as current for reference
        manager.callTarget(target);

        // Add the actual commands to the sequence
        addCommands(
            // Align
            new AutoAlignCommand(vision, drivebase, target.getTagId())
                .beforeStarting(() -> System.out.println("Starting alignment to tag " + target.getTagId()))
                .finallyDo((interrupted) -> System.out.println("Alignment " + (interrupted ? "interrupted" : "completed"))),

            // Elevator
            new SetElevatorLevelCommand(elevator, target.getLevel())
                .beforeStarting(() -> System.out.println("Moving elevator to level " + target.getLevel())),

            // Scoring
            new ESpitCMD(intakeMotor).withTimeout(1.0)
                .beforeStarting(() -> System.out.println("Executing scoring sequence")),

            // Mark as scored
            new InstantCommand(() -> {
                System.out.println("Marking target as scored: " + target);
                manager.markScored(target);
            })
        );

        // Debugging info
        System.out.println("Target details - Face: " + target.getFace()
            + " | Level: " + target.getLevel()
            + " | TagID: " + target.getTagId());

        System.out.println("TAG_IDS: " + Arrays.deepToString(Constants.Vision.TAG_IDS));
        logInitialization();
    }
   
    }
