// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState.TARGET;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {

  private SendableChooser<Command> autoChooser;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverCtrl = new CommandXboxController(0); // My driverCtrl
  private final Telemetry logger = new Telemetry(DriveConstants.MaxSpeed);

  public final Drivetrain drivetrain = TunerConstants.DriveTrain;
  public final RobotState robotState = RobotState.getInstance();
  public final Arm arm = new Arm();
  public final Intake intake = new Intake();
  public final Shooter shooter = new Shooter();
  public final Stage stage = new Stage();

  //TODO: Move to robotstate?
  private boolean isHarmonyClimbing = false;

  //TODO: add photon vision
  //TODO: add note detection
  //TODO: add auto intake
  //TODO: add auto paths
  //TODO: add advantagekit/scope
  //TODO: add Mechanism2d displays for subsystems
  //TODO: add leds
  //TODO: add controller rumble
  //TODO: add two stage joystick trigger functionality, i.e. auto intake when fully pulled, auto shoot when partially pulled
  //TODO: fusion of user and auto driving?
  //TODO: sendable chooser with filters
  //TODO: brownout override and accel settings

  
  private void configureBindings() {

    //Every loop of the periodic, pass the joystick values to the drivetrain
    drivetrain.setDefaultCommand(drivetrain.run(
        () -> drivetrain.setControllerInput(-driverCtrl.getLeftY(), -driverCtrl.getLeftX(), -driverCtrl.getRightX())));

    drivetrain.registerTelemetry(logger::telemeterize);

    Trigger readyToShoot = new Trigger(() -> arm.atGoal() && shooter.atGoal() && drivetrain.atGoal());
    Trigger hasNote = new Trigger(() -> stage.hasNote());
    
    // TODO: Add smart collection
    driverCtrl.leftTrigger(Constants.ControllerConstants.leftTriggerMin)
        .whileTrue(arm.setStateCommand(Arm.State.INTAKE) //Set arm goal to intake position
            .alongWith(Commands.waitUntil(arm::atGoal).andThen(stage.setStateCommand(Stage.State.INTAKE) //also wait until at position, then run stage
                .alongWith(intake.setStateCommand(Intake.State.INTAKE)))) //and intake
            .until(hasNote)); //until we have a note

    // Run the stage to shoot
    driverCtrl.rightTrigger().whileTrue(stage.setStateCommand(Stage.State.SHOOT));

    // Run the stage to shoot when arm and shooter are ready
    // TODO: check if this gets cancelled too early
    driverCtrl.rightBumper().whileTrue(Commands.waitUntil(arm::atGoal)
        .alongWith(Commands.waitUntil(shooter::atGoal))
        .andThen(stage.setStateCommand(Stage.State.SHOOT)));

    //Climb
    driverCtrl.y().whileTrue(Commands.either(arm.setStateCommand(Arm.State.HARMONY), arm.setStateCommand(Arm.State.CLIMB), () -> isHarmonyClimbing));

    //Subwoofer
    driverCtrl.a().whileTrue(arm.setStateCommand(Arm.State.SUBWOOFER)
            .alongWith(shooter.setStateCommand(Shooter.State.SUBWOOFER)));

    //Amp
    driverCtrl.b()
        .whileTrue(robotState.setTargetCommand(TARGET.AMP)
            .alongWith(arm.setStateCommand(Arm.State.AMP))
            .alongWith(shooter.setStateCommand(Shooter.State.AMP)));

    //Speaker
    driverCtrl.x()
        .whileTrue(robotState.setTargetCommand(TARGET.SPEAKER)
            .alongWith(arm.setStateCommand(Arm.State.LOOKUP))
            .alongWith(shooter.setStateCommand(Shooter.State.SHOOT)));

    //Feed
    driverCtrl.back()
        .whileTrue(robotState.setTargetCommand(TARGET.FEED)
            .alongWith(arm.setStateCommand(Arm.State.FEED))
            .alongWith(shooter.setStateCommand(Shooter.State.FEED)));

    //TODO: Remove when done with testing
    driverCtrl.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
      
    //Toggle whether we are harmony climbing or not
    driverCtrl.povDown().onTrue(Commands.runOnce(()-> isHarmonyClimbing = !isHarmonyClimbing));
    

  }

  public RobotContainer() {
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    }
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
}

}
