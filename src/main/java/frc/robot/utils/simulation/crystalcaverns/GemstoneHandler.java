package frc.robot.utils.simulation.crystalcaverns;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class GemstoneHandler {
    private final List<Pose3d> stagedGemstones = new ArrayList<>();
    // private boolean jeweleryFull = false;

    private static GemstoneHandler INSTANCE;

    public static GemstoneHandler getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new GemstoneHandler();
        }
        return INSTANCE;
    }

    private GemstoneHandler() {
        reset();
    }

    public void reset() {
        stagedGemstones.clear();
        stagedGemstones.add(FieldConstants.BLUE_GEMSTONE_STARTING_POSE);
        stagedGemstones.add(FieldConstants.RED_GEMSTONE_STARTING_POSE);
        // jeweleryFull = false;
    }

    public void periodic() {
        // simulateJewelry();
    }

    public Optional<Pose3d> intake(Pose3d intakePose) {
        var iterator = stagedGemstones.iterator();
        while (iterator.hasNext()) {
            Pose3d gemstone = iterator.next();
            if (checkTolerance(intakePose.minus(gemstone))) {
                iterator.remove();
                return Optional.of(gemstone);
            }
        }
        return Optional.empty();
    }

    // // collect launched gemstone projectiles
    // public void simulateJewelry() {
    //     if (jeweleryFull) return;

    //     var iterator = SimulatedArena.getInstance().gamePieceLaunched().iterator();
    //     while (iterator.hasNext()) {
    //         var gamePiece = iterator.next();

    //         if (!(gamePiece instanceof GemstoneOnFly)) {
    //             continue;
    //         }

    //         Pose3d pose = gamePiece.getPose3d();
    //         Translation2d translation = pose.getTranslation().toTranslation2d();

    //         boolean falling = gamePiece.getVelocity3dMPS().getZ() <= 0;
    //         boolean inHeightRange = pose.getZ() < FieldConstants.JEWELRY_MIN_Z;
    //         boolean inTarget = translation.getDistance(FieldConstants.CENTER_ORIGIN)
    //             < FieldConstants.JEWELRY_POS_TOLERANCE;
    //         boolean correctRotation = checkRotation(pose.getRotation());

    //         System.out.printf("falling %b, minz %b, translation %b, rot %b%n",falling,inHeightRange,inTarget,correctRotation);

    //         if (!falling || !inHeightRange || !inTarget || !correctRotation) {
    //             continue;
    //         }

    //         iterator.remove();
    //         jeweleryFull = true;

    //         stagedGemstones.add(new Pose3d(
    //             (pose.getX() + FieldConstants.CENTER_ORIGIN.getX()) / 2.0,
    //             (pose.getY() + FieldConstants.CENTER_ORIGIN.getY()) / 2.0,                
    //             Units.inchesToMeters(1.1188 + 0.5),
    //             new Rotation3d(0, 0, pose.getRotation().getZ())));
    //     }
    // }

    public void draw(List<Pose3d> poses) {
        poses.addAll(stagedGemstones);
    }

    private static boolean checkTolerance(Transform3d difference) {
        return difference.getTranslation().getNorm() < Units.inchesToMeters(12);
    }

    // private static boolean checkRotation(Rotation3d rotation) {
    //     double tilt = Math.hypot(rotation.getX(), rotation.getY());
    //     boolean withinTiltTolerance = tilt < FieldConstants.JEWELRY_ROT_TOLERANCE;

    //     Translation3d up = new Translation3d(0, 0, 1).rotateBy(rotation);
    //     boolean upright = up.getZ() > 0;

    //     return withinTiltTolerance && upright;
    // }
}
