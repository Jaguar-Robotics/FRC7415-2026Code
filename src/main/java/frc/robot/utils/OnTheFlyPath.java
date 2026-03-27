package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.FieldConstants;

import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

public class OnTheFlyPath {

    public static Command driveToPose(Supplier<Pose2d> currentPoseSupplier, Pose2d targetPose, PathConstraints constraints) {
        return Commands.defer(
            () -> buildPathCommand(currentPoseSupplier.get(), targetPose, constraints),
            Set.of()
        );
    }

    private static Command buildPathCommand(Pose2d currentPose, Pose2d targetPose, PathConstraints constraints) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        double midX = alliance == Alliance.Red
            ? FieldConstants.Trench.RedTrenchX
            : FieldConstants.Trench.BlueTrenchX;

        double midY = targetPose.getY() > FieldConstants.Field.HalfwayY
            ? FieldConstants.Trench.TopTrench
            : FieldConstants.Trench.BtmTrench;

        Rotation2d startToMid = new Rotation2d(Math.atan2(midY - currentPose.getY(), midX - currentPose.getX()));
        Rotation2d midToEnd   = new Rotation2d(Math.atan2(targetPose.getY() - midY, targetPose.getX() - midX));

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(currentPose.getTranslation(), startToMid),
            new Pose2d(new Translation2d(midX, midY), midToEnd),
            new Pose2d(targetPose.getTranslation(), midToEnd)
        );

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0.0, targetPose.getRotation())
        );

        path.preventFlipping = true;
        return AutoBuilder.followPath(path);
    }
}