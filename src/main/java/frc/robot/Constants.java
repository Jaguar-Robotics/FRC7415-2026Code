// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class Constants {

    
    public static abstract class ShooterConstants {
        public static final int ShooterLeaderID = 27; 
        public static final int ShooterFollowerID = 28;
        public static final int ShooterFollowerReversedID = 23; //
        public static final int ShooterFollowerReversed2ID = 24; //
        public static final AngularVelocity FastShot = RPM.of(3000); //HESHEL CHANGE ME FAST
        public static final AngularVelocity SlowShot = RPM.of(1500); //HESKEL CHANGE ME SLOW
        public static final Double RPSHardStop = 100.0; //kracken at 6k RPM
        public static final int RPSTolarance = 1;
        public static final double kS = 0.1; //volt to overcome static friction feedforawd
        public static final double kV = 0.12; //volts per rps
    }

    public static abstract class IntakeConstants {
        public static final int IntakeMotorID = 25; 
        public static final int IntakeFollowerReversedMotorID = 30; 
        public static final double FastIntake = 0.87; //used to be 1 but 0.87 is more effechient or sum
        public static final double SlowIntake = 0.5;
        public static final double SlowReverse = -0.3;
        public static final double FastReverse = -0.87;

        public static final int IntakeSlideMotorID = 35; //
        public static final Distance IntakeSlideOutSetPoint = Inches.of(13.3); //Changememaybe
        public static final Distance IntakeSlideMiddleSetPoint = Inches.of(3);
        public static Distance IntakeSlideInSetPoint = Inches.of(0); //change?
    }

    public static abstract class HopperConstants {
        public static final int HopperMotorID = 26; //
        public static final double FastRoll = 0.4; 
        public static final double SlowRoll = 0.2;
        public static final double FastOutRoll = -0.4;
        public static final double SlowOutRoll = -0.2;

    }

        public static abstract class IndexerConstants {
        public static final int HighIndexerMotorID = 37; //
        public static final int LowIndexerMotorID = 36; //
        public static final double FastRoll = 0.87;
        public static final double SlowRoll = 0.6;
        public static final double FastOutRoll = -0.5;
        public static final double SlowOutRoll = -0.3;

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
            public static final double xyP = 1;
            public static final double xyI = 0;
            public static final double xyD = 0;

            public static final double rotP = 0.85; //0.05
            public static final double rotI = 0;
            public static final double rotD = 0.05; //0.05

            public static final double TranslationDeadband = 0.1;
            public static final double RotationDeadband = 0.1;

            public static final double RotationalToleranceDegrees = 5.0;
        }
}