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
import frc.robot.Constants.FieldConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class RobotState {
    
    private static RobotState instance;

    private Pose2d robotPose = new Pose2d();

    @Getter
    @RequiredArgsConstructor
    public enum Target {
        NONE    (null, null),
        AMP     (FieldConstants.BLUE_AMP, FieldConstants.RED_AMP),
        SPEAKER (FieldConstants.BLUE_SPEAKER, FieldConstants.RED_SPEAKER),
        FEED    (FieldConstants.BLUE_FEED, FieldConstants.RED_FEED);

        private final Pose2d blueTargetPose;
        private final Pose2d redTargetPose;
    }

    @Getter
    @Setter
    private Target target = Target.NONE;

    double distanceToTarget; 
    /**
    * Creates a singleton RobotState.
    *
    * @return the arm subsystem instance.
    */
    public static RobotState getInstance() {
        if (instance == null) {
        instance = new RobotState();
        }

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
        speakerArmAngleMap.put(1.5, 12.71 -17.6);
        speakerArmAngleMap.put(2.0, 21.00-17.6);
        speakerArmAngleMap.put(2.5, 24.89-17.6);
        speakerArmAngleMap.put(3.0, 29.00-17.6);
        speakerArmAngleMap.put(3.5, 31.20-17.6);
        speakerArmAngleMap.put(4.0, 32.50-17.6);
        speakerArmAngleMap.put(4.5, 34.00-17.6);
        speakerArmAngleMap.put(5.0, 35.00-17.6);
    }
    
    private static final InterpolatingDoubleTreeMap ampArmAngleMap = new InterpolatingDoubleTreeMap();
    static {
        ampArmAngleMap.put(1.5, 12.71);
        ampArmAngleMap.put(2.0, 21.00);
        ampArmAngleMap.put(2.5, 24.89);
        ampArmAngleMap.put(3.0, 29.00);
        ampArmAngleMap.put(3.5, 31.20);
        ampArmAngleMap.put(4.0, 32.50);
        ampArmAngleMap.put(4.5, 34.00);
        ampArmAngleMap.put(5.0, 35.00);
    }
    
    private static final InterpolatingDoubleTreeMap feedArmAngleMap = new InterpolatingDoubleTreeMap();
    static {
        feedArmAngleMap.put(1.5, 12.71);
        feedArmAngleMap.put(2.0, 21.00);
        feedArmAngleMap.put(2.5, 24.89);
        feedArmAngleMap.put(3.0, 29.00);
        feedArmAngleMap.put(3.5, 31.20);
        feedArmAngleMap.put(4.0, 32.50);
        feedArmAngleMap.put(4.5, 34.00);
        feedArmAngleMap.put(5.0, 35.00);
    }
    
    public double getShotAngle() {
        if (target == Target.SPEAKER) {
            return speakerArmAngleMap.get(getDistanceToTarget());
        } else if (target == Target.AMP) {
            return ampArmAngleMap.get(getDistanceToTarget());
        } else if (target == Target.FEED) {
            return feedArmAngleMap.get(getDistanceToTarget());
        } else {
            return 0.0;
        } 
    }

    public void setRobotPose(Pose2d pose) {
        robotPose = pose;
    }

    public Command setTargetCommand(Target target) {
        return Commands.startEnd(() -> setTarget(target), () -> setTarget(Target.NONE))
                .withName("Set Robot Target: " + target.toString());
    }
}
