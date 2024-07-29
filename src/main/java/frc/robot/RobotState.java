// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class RobotState {
    private final Rotation2d ampAngle = new Rotation2d(-Math.PI / 2);

    public Rotation2d getAmpAngle() {
        return ampAngle;
    }

    private static final InterpolatingDoubleTreeMap speakerArmAngleMap = new InterpolatingDoubleTreeMap();
    static {
        speakerArmAngleMap.put(1.5, 12.71);
        speakerArmAngleMap.put(2.0, 21.0);
        speakerArmAngleMap.put(2.5, 24.89);
        speakerArmAngleMap.put(3.0, 29.00);
        speakerArmAngleMap.put(3.5, 31.20);
        speakerArmAngleMap.put(4.0, 32.50);
        speakerArmAngleMap.put(4.5, 34.00);
        speakerArmAngleMap.put(5.0, 35.00);

    }
    //TODO: Feed distance from speaker
    public double getShotAngle() {
        return speakerArmAngleMap.get(0.0);
    }

}
