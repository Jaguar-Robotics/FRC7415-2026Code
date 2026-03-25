package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        Rotation2d travelAngle = new Rotation2d(Math.atan2(dy, dx));

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(currentPose.getTranslation(), travelAngle),
            new Pose2d(targetPose.getTranslation(), travelAngle)
            //TODO DO THE POSE OF TRENCH HAVE 2 HAVE IT BE A VARIABLE
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