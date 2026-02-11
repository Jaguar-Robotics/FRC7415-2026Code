// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class Constants {

    public static abstract class ShooterConstants {
        public static final int ShooterLeaderID = 20; //all IDs should be changed  (I think theyre 40-43 and 0?)
        public static final int ShooterFollowerID = 21;
        public static final int ShooterFollowerReversedID = 22;
        public static final int ShooterFollowerReversed2ID = 23;
        public static final AngularVelocity FastShot = RPM.of(3000); //HESHEL CHANGE ME FAST
        public static final AngularVelocity SlowShot = RPM.of(1500); //HESKEL CHANGE ME SLOW
        public static final AngularVelocity SetRPMHardStop = RPM.of(3200);
        public static  double ShootingDistance = Feet.of(6).in(Meter); //6ft starting
        public static int RPMTolarance = 50;
    }

    public static abstract class IntakeConstants {
        public static final int IntakeMotorID = 31; 
        public static final int IntakeFollowerReversedMotorID = 30; 
        public static final double FastIntake = 1;
        public static final double SlowIntake = 0.5;
        public static final double SlowReverse = -0.3;
        public static final double FastReverse = -1;
    }

    public static abstract class HopperConstants {
        public static final int HopperMotorID = 32; //
        public static final double FastRoll = -0.3; 
        public static final double SlowRoll = -0.2;
        public static final double FastOutRoll = 0.8;
        public static final double SlowOutRoll = 0.4;

    }

        public static abstract class IndexerConstants {
        public static final int IndexerLowMotorID = 33; //
        public static final int IndexerHighMotorID = 34; //NOT RIGHTFAKE FIX NOW
        public static final double FastRoll = 1;
        public static final double SlowRoll = 0.6;
        public static final double FastOutRoll = -0.5;
        public static final double SlowOutRoll = -0.3;

    }

    public static abstract class ClimberConstants {
        public static final int ClimberMotorID = 32; //change me ts NOT a real motor
        public static final double ClimbSpeed = 0.7;
        public static final double DescendSpeed = -0.5;

        public static final Distance HighSetPoint = Inches.of(63.0);
        public static final Distance MiddleSetPoint = Inches.of(45.0);
        public static final Distance LowSetPoint = Inches.of(27.0);

        public static final Distance LowSetPointDown = Inches.of(23.0);
    }

        public static abstract class FieldConstants{
        public static final Pose3d redHubPose = new Pose3d(
            Units.Inches.of(468.56), 
            Units.Inches.of(158.32), 
            Units.Inches.of(72.0), 
            new Rotation3d()
        );
        
        public static final Pose3d blueHubPose = new Pose3d(
            Units.Inches.of(152.56+28), //152.56
            Units.Inches.of(158.32), //158.32
            Units.Inches.of(72.0), 
            new Rotation3d()
        );
    }
        public static abstract class DriveConstants{
            public static final double xyP = 10;
            public static final double xyI = 0;
            public static final double xyD = 0;

            public static final double rotP = 3;
            public static final double rotI = 0;
            public static final double rotD = 0.175;

            public static final double TranslationDeadband = 0.1;
            public static final double RotationDeadband = 0.1;

            public static final double RotationalToleranceDegrees = 5.0;
        }
}