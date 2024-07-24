// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverCtrl = new CommandXboxController(0); // My driverCtrl
  public final Drivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final Arm arm = new Arm();
  public final Intake intake = new Intake();
  public final Shooter shooter = new Shooter();
  public final Stage stage = new Stage();


  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverCtrl.getLeftY() * MaxSpeed) // Drive forward -Y
            .withVelocityY(-driverCtrl.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverCtrl.getRightX() * MaxAngularRate))); // Drive counterclockwise with negative X
                                                                             // (left)).ignoringDisable(true));

    drivetrain.registerTelemetry(logger::telemeterize);
    //TODO: Add smart collection
    driverCtrl.leftBumper().whileTrue(arm.setStateCommand(Arm.State.INTAKE)
        .alongWith(Commands.waitUntil(arm::atGoal)
            .andThen(stage.setStateCommand(Stage.State.INTAKE)
                .alongWith(intake.setStateCommand(Intake.State.INTAKE))))
        .until(() -> stage.hasNote()));

    driverCtrl.rightBumper().whileTrue(stage.setStateCommand(Stage.State.SHOOT));

    //TODO: add heading control from drivetrain
    driverCtrl.y().whileTrue(arm.setStateCommand(Arm.State.CLIMB));
    driverCtrl.b().whileTrue(arm.setStateCommand(Arm.State.AMP).alongWith(shooter.setStateCommand(Shooter.State.AMP)));
    driverCtrl.a().whileTrue(arm.setStateCommand(Arm.State.SUBWOOFER).alongWith(shooter.setStateCommand(Shooter.State.SUBWOOFER)));
    driverCtrl.x().whileTrue(arm.setStateCommand(Arm.State.LOOKUP).alongWith(shooter.setStateCommand(Shooter.State.SHOOT))); //Lookup shot
    driverCtrl.back().whileTrue(arm.setStateCommand(Arm.State.FEED).alongWith(shooter.setStateCommand(Shooter.State.FEED)));
    

  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }

}
