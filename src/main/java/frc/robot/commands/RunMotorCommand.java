
// package frc.robot.commands;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.RunMotorSub;

// public class RunMotorCommand extends Command {

//     private final RunMotorSub runMotorSub;
//     private final DoubleSupplier speed;

//     public RunMotorCommand(RunMotorSub runMotorSub, DoubleSupplier speed) {
//         this.runMotorSub = runMotorSub;
//         this.speed = speed;
//         addRequirements(runMotorSub);
//     }

//     @Override
//     public void execute() {
//         double Speed = speed.getAsDouble();
//         runMotorSub.runMotor(Speed); // Now it actually controls the motor
//     }

//     @Override
//     public void end(boolean interrupted) {
//         runMotorSub.stop();
//     }

//     @Override
//     public boolean isFinished() {
//         return false; // Runs indefinitely unless interrupted
//     }
// }
