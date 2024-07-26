// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {


  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverCtrl = new CommandXboxController(0); // My driverCtrl
  public final Drivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain


            

  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final RobotState robotState = new RobotState();
  public final Arm arm = new Arm();
  public final Intake intake = new Intake();
  public final Shooter shooter = new Shooter();
  public final Stage stage = new Stage();
  


  private void configureBindings() {
    SmartDashboard.putData(CommandScheduler.getInstance());
    drivetrain.setDefaultCommand(drivetrain.run(() -> drivetrain.setControllerInput(driverCtrl.getLeftX(), driverCtrl.getLeftY(), driverCtrl.getRightX()))
    .withName("Pass joystick to drivetrain"));


    drivetrain.registerTelemetry(logger::telemeterize);
    //TODO: Add smart collection
    driverCtrl.leftBumper().whileTrue(arm.setStateCommand(Arm.State.INTAKE)
        .alongWith(Commands.waitUntil(arm::atGoal)
            .andThen(stage.setStateCommand(Stage.State.INTAKE)
                .alongWith(intake.setStateCommand(Intake.State.INTAKE))))
        .until(() -> stage.hasNote()));

    driverCtrl.rightBumper().whileTrue(stage.setStateCommand(Stage.State.SHOOT));

    driverCtrl.y().whileTrue(arm.setStateCommand(Arm.State.CLIMB));

    driverCtrl.b()
        .whileTrue(arm.setStateCommand(Arm.State.AMP)
        .alongWith(shooter.setStateCommand(Shooter.State.AMP))
        .alongWith(Commands.startEnd(() -> drivetrain.setHeadingAngle(robotState.getAmpAngle()),drivetrain::clearHeadingAngle))
        .alongWith(drivetrain.setStateCommand(Drivetrain.State.HEADING)));

    driverCtrl.a().whileTrue(arm.setStateCommand(Arm.State.SUBWOOFER).alongWith(shooter.setStateCommand(Shooter.State.SUBWOOFER)));

    driverCtrl.x().whileTrue(arm.setStateCommand(Arm.State.LOOKUP).alongWith(shooter.setStateCommand(Shooter.State.SHOOT))); //Lookup shot
    
    driverCtrl.back().whileTrue(arm.setStateCommand(Arm.State.FEED).alongWith(shooter.setStateCommand(Shooter.State.FEED)));

    driverCtrl.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    

  }

  public RobotContainer() {
    configureBindings();

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    }
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }

}
