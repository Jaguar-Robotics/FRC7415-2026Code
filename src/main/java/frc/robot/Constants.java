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
        public static final int ShooterLeaderID = 20; 
        public static final int ShooterFollowerID = 21;
        public static final int ShooterFollowerReversedID = 22; //
        public static final int ShooterFollowerReversed2ID = 23; //
        public static final AngularVelocity FastShot = RPM.of(3000); //HESHEL CHANGE ME FAST
        public static final AngularVelocity SlowShot = RPM.of(1500); //HESKEL CHANGE ME SLOW
        public static final Double RPSHardStop = 80.0; 
        public static final int RPSTolarance = 1;
        public static final double kS = 0.3; //volt to overcome static friction feedforawd TUNED ALR
        public static final double kV = 0.12; //volts per rps
    }

    public static abstract class IntakeConstants {
        public static final int IntakeMotorID = 30; 
        public static final int IntakeFollowerReversedMotorID = 31; 
        public static final double FastIntake = 0.87; //used to be 1 but 0.87 is more effechient or sum
        public static final double SlowIntake = 0.5;
        public static final double SlowReverse = -0.3;
        public static final double FastReverse = -0.87;

        public static final int IntakeSlideMotorID = 35; 
        public static final double IntakeSlideOutSetPoint = 25.5; //In ROTATIONS
        public static final double IntakeSlideMiddleSetPoint = 9.75;
        public static final double IntakeSlideInSetPoint = 0; 
        public static final double SafeOutPosition = 13; //in rotations

        public static final double IntakeSlideOutHardStop = 25.5; //rotations; Used for re-zero out

        public static final double PositionTolerance = 0.3; //in degreee
       
    }

    public static abstract class HopperConstants {
        public static final int HopperMotorID = 32; //
        public static final double FastRoll = 0.4; 
        public static final double SlowRoll = 0.3; //0.2
        public static final double FastOutRoll = -0.4;
        public static final double SlowOutRoll = -0.2;

    }

        public static abstract class IndexerConstants {
        public static final int HighIndexerMotorID = 34; //
        public static final int LowIndexerMotorID = 33; //
        public static final double FastRoll = -0.87;
        public static final double SlowRoll = -0.6;
        public static final double FastOutRoll = 0.87;
        public static final double SlowOutRoll = 0.3;

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

            public static final double RotationalToleranceDegrees = 3.0;
        }
}