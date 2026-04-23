package frc.robot.utils;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Zones {

    public static List<Pose2d> rectToPoses(Rectangle2d rect) {
        Pose2d center = rect.getCenter();
        double halfX = rect.getXWidth() / 2.0;
        double halfY = rect.getYWidth() / 2.0;

        Translation2d c = center.getTranslation();
        Rotation2d rot = center.getRotation();

        // Compute corners relative to center, then rotate by the rectangle's rotation
        Translation2d topLeft     = c.plus(new Translation2d(-halfX,  halfY).rotateBy(rot));
        Translation2d topRight    = c.plus(new Translation2d( halfX,  halfY).rotateBy(rot));
        Translation2d bottomRight = c.plus(new Translation2d( halfX, -halfY).rotateBy(rot));
        Translation2d bottomLeft  = c.plus(new Translation2d(-halfX, -halfY).rotateBy(rot));

        return List.of(
            new Pose2d(topLeft,     new Rotation2d()),
            new Pose2d(topRight,    new Rotation2d()),
            new Pose2d(bottomRight, new Rotation2d()),
            new Pose2d(bottomLeft,  new Rotation2d()),
            new Pose2d(topLeft,     new Rotation2d())  // close the loop
        );
    }
}