package frc.robot.utils.simulation.crystalcaverns;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

// Crystal Station
public class MineSim {
    public static double lastDropTime = 0;

    public static Pose2d getNearestStation(Pose2d curPose) {
        return curPose.nearest(FieldConstants.MINES);
    }

    public static void drop(Pose2d stationPose, boolean addRandomness) {
        if (!canDrop()) return;

        GamePieceProjectile gamePiece;

        if (addRandomness) {
            gamePiece = new CrystalOnFly(
                    stationPose.getTranslation(),
                    new Translation2d(0, Math.random() * 1.5 - 0.75),
                    new ChassisSpeeds(),
                    stationPose.getRotation().plus(Rotation2d.fromDegrees(Math.random() * 30 - 15)),
                    Meters.of(1),
                    MetersPerSecond.of(2.5 + Math.random() * 5),
                    Degrees.of(-55 + Math.random() * 30));
        } else {
            gamePiece = new CrystalOnFly(
                    stationPose.getTranslation(),
                    Translation2d.kZero,
                    new ChassisSpeeds(),
                    stationPose.getRotation(),
                    Meters.of(1),
                    MetersPerSecond.of(5),
                    Degrees.of(-55));
        }

        SimulatedArena.getInstance().addGamePieceProjectile(gamePiece);
        lastDropTime = Timer.getFPGATimestamp();
    }

    public static boolean canDrop() {
        return DriverStation.isEnabled()
                && Timer.getFPGATimestamp() - lastDropTime > 3;
    }
}
