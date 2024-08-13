// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

/** Add your docs here. */
public class RobotState {
    private static RobotState instance;
    private Pose2d robotPose = new Pose2d();

    @RequiredArgsConstructor
    @Getter
    public enum TARGET {
        NONE(null,null),
        SPEAKER(Constants.FieldConstants.BLUE_SPEAKER,Constants.FieldConstants.RED_SPEAKER),
        AMP(Constants.FieldConstants.BLUE_AMP,Constants.FieldConstants.RED_AMP),
        FEED(Constants.FieldConstants.BLUE_FEED,Constants.FieldConstants.RED_FEED);

        private final Pose2d blueTargetPose;
        private final Pose2d redTargetPose;

    }

    @Getter
    @Setter
    private TARGET target = TARGET.NONE;


    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
      }


      public Rotation2d getAngleOfTarget() {
          return (DriverStation.getAlliance().get() == Alliance.Blue) ? target.blueTargetPose.getRotation() : target.redTargetPose.getRotation();
      }

      //TODO: need to invert
      public Rotation2d getAngleToTarget() {
          return robotPose.getTranslation()
                  .minus((DriverStation.getAlliance().get() == Alliance.Blue) ? target.blueTargetPose.getTranslation(): target.redTargetPose.getTranslation())
                  .getAngle();
      }

      public double getDistanceToTarget() {
          return robotPose.getTranslation().getDistance(
                  (DriverStation.getAlliance().get() == Alliance.Blue) ? target.blueTargetPose.getTranslation(): target.redTargetPose.getTranslation());
      }




    private static final InterpolatingDoubleTreeMap speakerArmAngleMap = new InterpolatingDoubleTreeMap();
    static {
        speakerArmAngleMap.put(1.5, 12.71);
        speakerArmAngleMap.put(2.0, 21.00);
        speakerArmAngleMap.put(2.5, 24.89);
        speakerArmAngleMap.put(3.0, 29.00);
        speakerArmAngleMap.put(3.5, 31.20);
        speakerArmAngleMap.put(4.0, 32.50);
        speakerArmAngleMap.put(4.5, 34.00);
        speakerArmAngleMap.put(5.0, 35.00);

    }

    public double getShotAngle() {
        if (target == TARGET.SPEAKER) {
            return speakerArmAngleMap.get(getDistanceToTarget());
        } else {
            return 0.0;
        } 
    }

    public void setRobotPose(Pose2d pose) {
        robotPose = pose;
    }

    public Command setTargetCommand(TARGET target) {
        return Commands.startEnd(() -> setTarget(target), () -> setTarget(TARGET.NONE))
                .withName("Set Robot Target: " + target.toString());
    }

}
