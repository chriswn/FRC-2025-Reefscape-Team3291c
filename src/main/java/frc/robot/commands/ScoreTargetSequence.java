package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCMDs.SetElevatorLevelCommand;
import frc.robot.commands.IntakeMotorCMDs.ESpitCMD;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.intake.IntakeMotorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.Optional;

public class ScoreTargetSequence extends SequentialCommandGroup {

    public ScoreTargetSequence(
        VisionSubsystem vision,
        SwerveSubsystem drivebase,
        ElevatorSubsystem elevator,
        IntakeMotorSubsystem intakeMotor,
        ScoringTargetManager manager
    ) {
        if (manager == null) {
            throw new IllegalStateException("[ERROR] ScoringTargetManager is null!");
        }

        ScoringTarget target = manager.getCurrentTarget().orElse(manager.getNextTarget());
        if (target == null) {
            throw new IllegalStateException("[ERROR] No scoring target available.");
        }

        manager.callTarget(target);
        System.out.println("[ScoreTargetCommand] Target selected: " + target);

        addCommands(
            new AutoAlignCommand(vision, drivebase, () -> Optional.of(target))
                .beforeStarting(() -> System.out.println("[ScoreTargetCommand] Starting ALIGN to TagID: " + target.getTagId())),

            new SetElevatorLevelCommand(elevator, target.getLevel())
                .beforeStarting(() -> System.out.println("[ScoreTargetCommand] Alignment complete. Moving elevator...")),

            new ESpitCMD(intakeMotor).withTimeout(1.0)
                .beforeStarting(() -> System.out.println("[ScoreTargetCommand] Elevator in position. Spitting..."))
                .andThen(() -> {
                    System.out.println("[ScoreTargetCommand] Spitting complete. Marking scored.");
                    manager.markScored(target);
                })



                
        );
    }
}
