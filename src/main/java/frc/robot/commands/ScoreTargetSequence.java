package frc.robot.commands;

import java.util.Arrays;
import java.util.Optional;

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

public class ScoreTargetSequence extends SequentialCommandGroup {

    private final ScoringTarget target;

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

        // ✅ Safely hold selected target ONCE
        target = manager.getCurrentTarget().orElse(manager.getNextTarget());
        if (target == null) {
            System.err.println("[ERROR] No scoring targets available!");
            throw new IllegalStateException("No target available for scoring.");
        }

        // ✅ Set current target BEFORE using it in commands
        manager.callTarget(target);
        System.out.println("Selected target: " + target);

        // ✅ Add commands using that exact target instance
        addCommands(
    new AutoAlignCommand(vision, drivebase,  () -> Optional.of(target))
                .beforeStarting(() -> System.out.println("Starting alignment to tag " + target.getTagId()))
                .finallyDo((interrupted) -> System.out.println("Alignment " + (interrupted ? "interrupted" : "completed"))),
             //   .withTimeout(20.0), // Give it more time to align

            new SetElevatorLevelCommand(elevator, target.getLevel())
                .beforeStarting(() -> System.out.println("Moving elevator to level " + target.getLevel())),

            new ESpitCMD(intakeMotor).withTimeout(1.0)
                .beforeStarting(() -> System.out.println("Executing scoring sequence"))
                .finallyDo((interrupted) -> System.out.println("Scoring " + (interrupted ? "interrupted!" : "done"))),

            new InstantCommand(() -> {
                System.out.println("Marking target as scored: " + target);
                manager.markScored(target);
            })
        );

        // Extra debug
        System.out.println("Target details - Face: " + target.getFace()
            + " | Level: " + target.getLevel()
            + " | TagID: " + target.getTagId());

        System.out.println("TAG_IDS: " + Arrays.deepToString(Constants.Vision.TAG_IDS));
        logInitialization();
    }
    private void logInitialization() {
        System.out.println("[ScoreTargetSequence] Initialized");

        SmartDashboard.putString("ScoreTargetSequence/Status", "Initialized");
        SmartDashboard.putString("ScoreTargetSequence/Target",
            "Face: " + target.getFace() +
            ", Level: " + target.getLevel() +
            ", TagID: " + target.getTagId());
    }
    }
