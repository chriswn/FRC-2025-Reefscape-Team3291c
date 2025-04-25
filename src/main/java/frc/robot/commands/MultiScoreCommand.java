package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.intake.IntakeMotorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.ElevatorCMDs.SetElevatorLevelCommand;
import frc.robot.commands.IntakeMotorCMDs.ESpitCMD;
import frc.robot.commands.AutoAlignCommand;
public class MultiScoreCommand extends SequentialCommandGroup {
    public MultiScoreCommand(
      VisionSubsystem vision,
      SwerveSubsystem drivebase,
      ElevatorSubsystem elevator,
      IntakeMotorSubsystem intake,
      ScoringTargetManager manager
    ) {
      addCommands(
        new RepeatCommand(
          new ProxyCommand(() -> {
            ScoringTarget target = manager.getNextTarget();
            if (target == null) {
              return Commands.none(); // gracefully finish
            }
  
            manager.callTarget(target);
            SmartDashboard.putString("Picked target", target.toString());
  
            return new SequentialCommandGroup(
              // Align to that target
              new AutoAlignCommand( vision,  drivebase, (Supplier<Optional<ScoringTarget>>) () -> Optional.of(target))
                .beforeStarting(() ->
                  System.out.println("[MultiScore] Aligning to TagID: " + target.getTagId())
                ),
  
              // Move elevator
              new SetElevatorLevelCommand(elevator, target.getLevel())
                .beforeStarting(() ->
                  System.out.println("[MultiScore] Elevator to level " + target.getLevel())
                ),
  
              // Score
              new ESpitCMD(intake).withTimeout(1.0),
  
              // Mark scored
              Commands.runOnce(() -> {
                manager.markScored(target);
                System.out.println("[MultiScore] Scored: " + target);
              })
            );
          })
        ).until(() -> manager.getNextTarget() == null)
      );
    }
  }
  