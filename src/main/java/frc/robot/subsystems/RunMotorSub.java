// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.LimelightHelpers;
// import frc.robot.subsystems.LimelightHelpers.LimelightResults;

// public class RunMotorSub extends SubsystemBase {

//     private final SparkMax motor;

//     public RunMotorSub() {
//         motor = new SparkMax(23, MotorType.kBrushless);
//     }

//     @Override
//     public void periodic() {
//         // This method will be called once per scheduler run
//     }

//     public void runMotor(double speed) {
//         motor.setVoltage(speed);
//     }

//     public void stop() {
//         motor.setVoltage(0);
//     }
// }