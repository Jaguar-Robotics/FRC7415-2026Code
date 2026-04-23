// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Constants {

    
    public static abstract class ShooterConstants {
        public static final int ShooterLeaderID = 20; 
        public static final int ShooterFollowerID = 21;
        public static final int ShooterFollowerReversedID = 22; //
        public static final int ShooterFollowerReversed2ID = 23; //
        public static final double FastShot = 100; // IN RPS
        public static final double SlowShot = 20; //HESKEL CHANGE ME SLOW
        public static final double RPSHardStop = 200.0; 
        public static final double RPSTolarance = 1;
        public static final double kS = 0.28; //0.28
        public static final double kV = 0.13; //0.13 
    }

    public static abstract class IntakeConstants {
        public static final int IntakeMotorID = 30; 
        public static final int KickerMotorID = 45; //change ts
        public static final int IntakeFollowerReversedMotorID = 31; 
        public static final double FastIntake = 0.87;
        public static final double SlowIntake = 0.67;
        public static final double SlowReverse = -0.3;
        public static final double FastReverse = -0.87;

        public static final int IntakeSlideMotorID = 35; 
        public static final double IntakeSlideOutSetPoint = 23.6; //In ROTATIONS 25
        public static final double IntakeSlideMiddleSetPoint = 10;
        public static final double IntakeSlideInSetPoint = 0; 

        public static final double IntakeSlideOutHardStop = 23.7; //rotations; Used for re-zero out

        public static final double PositionTolerance = 0.3; //in degreee
       
    }

    public static abstract class HopperConstants {
        public static final int HopperMotorID = 32; //
        public static final double FastRoll = 0.7415; //used in SS
        public static final double SlowRoll = 0.3; //0.2
        public static final double FastOutRoll = -0.4;
        public static final double SlowOutRoll = -0.2;

    }

        public static abstract class IndexerConstants {
        public static final int HighIndexerMotorID = 34; //
        public static final int LowIndexerMotorID = 33; //
        public static final double FastRoll = -1;
        public static final double SlowRoll = -0.6;
        public static final double FastOutRoll = 1;
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
        public static final Pose2d blueTargetHighPose = new Pose2d(
            Units.Inches.of(10),
            Units.Inches.of(265), 
            new Rotation2d()
        );

        public static final Pose2d blueTargetLowPose = new Pose2d(
            Units.Inches.of(10),
            Units.Inches.of(50), 
            new Rotation2d()
        );

        public static final Pose2d redTargetHighPose = new Pose2d(
            Units.Inches.of(640), 
            Units.Inches.of(265), 
            new Rotation2d()
        );
        
        public static final Pose2d redTargetLowPose = new Pose2d(
            Units.Inches.of(640), 
            Units.Inches.of(50), 
            new Rotation2d()
        ); 

        public static final  Rectangle2d RedTower = new Rectangle2d(new Pose2d(0.5, 3.75, new Rotation2d()), 1.0, 2.0);
        public static final  Rectangle2d BlueTower = new Rectangle2d(new Pose2d(16, 4.35, new Rotation2d()), 1.0, 2.0);

    }
public static abstract class DriveConstants {
    public static final double xyP = 20;
    public static final double xyI = 0;
    public static final double xyD = 0.05;

    public static double rotP = 0.85; // removed 'final'
    public static double rotI = 0;
    public static double rotD = 0.05; // removed 'final'

    public static final double TranslationDeadband = 0.1;
    public static final double RotationDeadband = 0.1;

    public static final double RotationalToleranceDegrees = 3.5;
    public static final double RotationalToleranceDegreesAUTO = 6.0;
}
}