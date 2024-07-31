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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

  private boolean isHarmonyClimbing = false;

  
  
  private void configureBindings() {
    robotState.getShotAngle();

    drivetrain.setDefaultCommand(drivetrain.run(
        () -> drivetrain.setControllerInput(driverCtrl.getLeftX(), driverCtrl.getLeftY(), driverCtrl.getRightX())));

    drivetrain.registerTelemetry(logger::telemeterize);
    
    // TODO: Add smart collection
    driverCtrl.leftTrigger(Constants.ControllerConstants.leftTriggerMin)
        .whileTrue(arm.setStateCommand(Arm.State.INTAKE)
            .alongWith(Commands.waitUntil(arm::atGoal).andThen(stage.setStateCommand(Stage.State.INTAKE)
                .alongWith(intake.setStateCommand(Intake.State.INTAKE))))
            .until(() -> stage.hasNote()));

/*     driverCtrl.leftTrigger(Constants.ControllerConstants.leftTriggerMid).whileTrue(
      Commands.startEnd(() -> drivetrain.setHeadingAngle(robotState.getAmpAngle()),drivetrain::clearHeadingAngle)); */

    driverCtrl.rightTrigger().whileTrue(stage.setStateCommand(Stage.State.SHOOT));

    //TODO: Add Harmony climb
    driverCtrl.y().whileTrue(arm.setStateCommand(Arm.State.CLIMB));

    driverCtrl.a().whileTrue(arm.setStateCommand(Arm.State.SUBWOOFER).alongWith(shooter.setStateCommand(Shooter.State.SUBWOOFER)));

    driverCtrl.b()
        .whileTrue(robotState.setTargetCommand(TARGET.AMP)
            .alongWith(arm.setStateCommand(Arm.State.AMP))
            .alongWith(shooter.setStateCommand(Shooter.State.AMP))
            .alongWith(drivetrain.setStateCommand(Drivetrain.State.CARDINAL)));

    driverCtrl.x()
        .whileTrue(robotState.setTargetCommand(TARGET.SPEAKER)
            .alongWith(arm.setStateCommand(Arm.State.LOOKUP))
            .alongWith(shooter.setStateCommand(Shooter.State.SHOOT))
            .alongWith(drivetrain.setStateCommand(Drivetrain.State.HEADING))); // Lookup shot

    driverCtrl.back()
        .whileTrue(robotState.setTargetCommand(TARGET.FEED)
            .alongWith(arm.setStateCommand(Arm.State.FEED))
            .alongWith(shooter.setStateCommand(Shooter.State.FEED))
            .alongWith(drivetrain.setStateCommand(Drivetrain.State.HEADING)));

    //TODO: Remove when done with testing
    driverCtrl.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
      
    //Toggle whether we are harmony climbing or not
    driverCtrl.povDown().onTrue(Commands.either(Commands.runOnce(()-> isHarmonyClimbing = false) ,Commands.runOnce(()-> isHarmonyClimbing = true) , () -> isHarmonyClimbing));
    

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
