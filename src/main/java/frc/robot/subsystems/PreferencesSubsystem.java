// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.


// package frc.robot.subsystems;


// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import edu.wpi.first.wpilibj.Preferences;


// public class PreferencesSubsystem extends SubsystemBase {


//   public double launchSpeed = Constants.launchSpeed;
//   public double ejectSpeed = Constants.intake.ejectSpeed;
//   public double intakeSpeed = Constants.intake.intakeSpeed;
//   public double groundAngle = Constants.intake.groundAngle;
//   public double stowAngle = Constants.intake.stowAngle;
//   public double sourceAngle = Constants.intake.sourceAngle;
//   public double ampAngle = Constants.intake.ampAngle;
//   public double maxPivotVoltage = Constants.intake.maxPivotVoltage;


//   public PreferencesSubsystem() {
//     Preferences.initDouble("launchSpeed", launchSpeed);
//     Preferences.initDouble("ejectSpeed", ejectSpeed);
//     Preferences.initDouble("intakeSpeed", intakeSpeed);
//     Preferences.initDouble("groundAngle", groundAngle);
//     Preferences.initDouble("stowAngle", stowAngle);
//     Preferences.initDouble("sourceAngle", sourceAngle);
//     Preferences.initDouble("ampAngle", ampAngle);
//     Preferences.initDouble("maxPivotVoltage", maxPivotVoltage);
//   }


//   /** Creates a new Preferences. */
//   public void setPreferences() {
//     if (Preferences.getDouble("launchSpeed", launchSpeed) != launchSpeed) {
//       launchSpeed = Preferences.getDouble("launchSpeed", launchSpeed);
//     }


//     if (Preferences.getDouble("ejectSpeed", ejectSpeed) != ejectSpeed) {
//       ejectSpeed = Preferences.getDouble("ejectSpeed", ejectSpeed);
//     }


//     if (Preferences.getDouble("intakeSpeed", intakeSpeed) != intakeSpeed) {
//       intakeSpeed = Preferences.getDouble("intakeSpeed", intakeSpeed);
//     }


//     if (Preferences.getDouble("groundAngle", groundAngle) != groundAngle) {
//       groundAngle = Preferences.getDouble("groundAngle", groundAngle);
//     }


    if (Preferences.getDouble("stowAngle", stowAngle) != stowAngle) {
      stowAngle = Preferences.getDouble("stowAngle", stowAngle);
    }


//     if (Preferences.getDouble("sourceAngle", sourceAngle) != sourceAngle) {
//       sourceAngle = Preferences.getDouble("sourceAngle", sourceAngle);
//     }


//     if (Preferences.getDouble("ampAngle", ampAngle) != ampAngle) {
//       ampAngle = Preferences.getDouble("ampAngle", ampAngle);
//     }
//     if (Preferences.getDouble("maxPivotVoltage", maxPivotVoltage) != maxPivotVoltage) {
//       maxPivotVoltage = Preferences.getDouble("maxPivotVoltage", maxPivotVoltage);
//     }
//   }


//   @Override
//   public void periodic() {
//     setPreferences();
//   }
// }

