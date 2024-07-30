// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

/** Add your docs here. */
public class RobotState {
    private static RobotState instance;
    private Pose2d robotPose = new Pose2d();


    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
      }

    public Rotation2d getAngleToAmp() {
        return FieldConstants.ampAngle;
    }

    public boolean isRed() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    }

    public double getDistanceToSpeaker() {
        return robotPose.getTranslation().getDistance(isRed() ? FieldConstants.RED_SPEAKER.getTranslation() : FieldConstants.BLUE_SPEAKER.getTranslation());
    }

    public Rotation2d getAngleToSpeaker() {
        return robotPose.getTranslation().minus(isRed() ? FieldConstants.RED_SPEAKER.getTranslation() : FieldConstants.BLUE_SPEAKER.getTranslation()).getAngle();
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

    //TODO: Feed distance from speaker
    public double getShotAngle() {
        return speakerArmAngleMap.get(getDistanceToSpeaker());
    }

    public void setRobotPose(Pose2d pose) {
        robotPose = pose;
    }

}
