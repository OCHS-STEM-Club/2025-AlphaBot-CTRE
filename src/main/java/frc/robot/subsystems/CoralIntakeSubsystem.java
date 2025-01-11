// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.jni.CANSparkJNI;
// import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CoralIntakeSubsystem extends SubsystemBase {
  /** Creates a new coralIntakeSubsystem. */
  private SparkMax coralIntake;
  private SparkMaxConfig motorConfig;
  
  public CoralIntakeSubsystem() {
    coralIntake = new SparkMax(18, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();

    motorConfig.idleMode(IdleMode.kBrake);

    coralIntake.configure(motorConfig, null, null);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Coral Intake Motor", coralIntake.get());

  }
  
  public void intakeIn() {
    coralIntake.set(0.5);
  }
  public void intakeOff() {
    coralIntake.set(0);
  }
  public void intakeOut() {
    coralIntake.set(-0.5);
  }
}
