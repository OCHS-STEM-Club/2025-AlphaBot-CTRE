// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Coral_Intake.Coral_Intake_Cmd;
import frc.robot.commands.Coral_Intake.Coral_Outake_Cmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class RobotContainer {

    // Chooser Definitions
    private final SendableChooser<Command> autoChooser;
    private SendableChooser<Double> speedChooser = new SendableChooser<>();

    // Controller Definitions
    private final CommandXboxController m_driverController = new CommandXboxController(DriveConstants.kDriverControllerPort);
    
    // Subsystem Definitions
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final CoralIntakeSubsystem m_coralIntakeSubsystem = new CoralIntakeSubsystem();

    // Commands definitions
    Coral_Intake_Cmd m_coralIntakeCmd = new Coral_Intake_Cmd(m_coralIntakeSubsystem);
    Coral_Outake_Cmd m_coralOutakeCmd = new Coral_Outake_Cmd(m_coralIntakeSubsystem);

    // Swerve Requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MAX_SPEED * DriveConstants.TRANSLATION_DEADBAND)
            .withRotationalDeadband(DriveConstants.MAX_ANGULAR_RATE * DriveConstants.ROTATION_DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Telemetry defintions
    private final Telemetry logger = new Telemetry(DriveConstants.MAX_SPEED);

    // Main Constructor
    public RobotContainer() {
        // Auto Chooser Definitions
        autoChooser = AutoBuilder.buildAutoChooser();

        // Adjustable speed Chooser
        speedChooser.addOption("100%", 1.0);
        speedChooser.addOption("95%", 0.95);
        speedChooser.addOption("90%", 0.9);
        speedChooser.addOption("85%", 0.85);
        speedChooser.addOption("80%", 0.8);
        speedChooser.addOption("75%", 0.75);
        speedChooser.setDefaultOption("70%", 0.7);
        speedChooser.addOption("65%", 0.65);
        speedChooser.addOption("60%", 0.6);
        speedChooser.addOption("55%", 0.55);
        speedChooser.addOption("50%", 0.5);
        speedChooser.addOption("45%", 0.45);
        speedChooser.addOption("40%", 0.4);
        speedChooser.addOption("35%", 0.35);
        speedChooser.addOption("30%", 0.3);
        speedChooser.addOption("25%", 0.25);
        speedChooser.addOption("0%", 0.0);
        
        // Put chooser on dashboard
        SmartDashboard.putData("Speed Limit", speedChooser);

        configureBindings();

        // Put Auto Chooser On Dashboard
        SmartDashboard.putData("Autos", autoChooser);
    }

    private void configureBindings() {

        // New Speed method called for adjustable speed chooser
        newSpeed();

        // Swerve Bindings
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driverController.getLeftY() * DriveConstants.MAX_SPEED *0.5)
                    .withVelocityY(-m_driverController.getLeftX() * DriveConstants.MAX_SPEED *0.5)
                    .withRotationalRate(-m_driverController.getRightX() * DriveConstants.MAX_ANGULAR_RATE *0.5)
            )
        );

        // Creep button
        m_driverController.leftBumper().onTrue(Commands.runOnce(() ->
            DriveConstants.MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * DriveConstants.CREEP_SPEED));
    
        // Refresh button for speed chooser
        m_driverController.leftBumper().onFalse(Commands.runOnce(() ->
            DriveConstants.MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * speedChooser.getSelected()));

        // joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));

        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.leftBumper().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.rightBumper().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.leftTrigger().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.rightTrigger().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset GYRO
        m_driverController.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Intake CMD
        m_driverController.leftTrigger().whileTrue(
            m_coralIntakeCmd
        );

        // Outake CMD
        m_driverController.rightTrigger().whileTrue(
            m_coralOutakeCmd
        );

        // Register Telemetry
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void newSpeed() {
        DriveConstants.LAST_SPEED = speedChooser.getSelected();
        DriveConstants.MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * DriveConstants.LAST_SPEED;
      }

    // Autonomous chooser method
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
