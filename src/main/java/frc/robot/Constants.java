// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class Constants {
        public static abstract class ExampleSubsystemConstants{
            public static final int exampleMotorID = 67; 
            public static final double fastSpeed = 1;
            public static final double slowSpeed = 0;
        }

        public static abstract class DriveConstants{
            public static final double TranslationDeadband = 0.1;
            public static final double RotationDeadband = 0.1;

            public static final double RotationalToleranceDegrees = 3.5;
            public static final double RotationalToleranceDegreesAUTO = 6.0;
        }
}