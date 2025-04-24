package frc.robot.commands;

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

/**
 * Full sequence to call next scoring target, align to its AprilTag,
 * move elevator, and score the coral.
 */
public class ScoreTargetSequence extends SequentialCommandGroup {
    public ScoreTargetSequence(
        VisionSubsystem vision,
        SwerveSubsystem drivebase,
        ElevatorSubsystem elevator,
        IntakeMotorSubsystem intakeMotor,
        ScoringTargetManager manager

    ) {


        
        // Pick current or next target
        ScoringTarget target = manager.getCurrentTarget().orElse(manager.getNextTarget());
        System.out.println("Scoring Target Called: " + target);


        if (target == null) {
            throw new IllegalStateException("No target available for scoring.");
        }


    

        // Save the target as current for reference
        manager.callTarget(target);

       

        addCommands(
            // Align to the target tag
            new AutoAlignCommand(vision, drivebase, target.getTagId()),
            // Move elevator to the target level
            new SetElevatorLevelCommand(elevator, target.getLevel()),
            // Score coral by spitting it
            new ESpitCMD(intakeMotor).withTimeout(1.0),

            // 🧠 NEW: Mark the target as scored using ReefMap
            new InstantCommand(() -> manager.markScored(target))
        );
    }
}
