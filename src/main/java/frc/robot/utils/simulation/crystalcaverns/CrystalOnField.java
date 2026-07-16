package frc.robot.utils.simulation.crystalcaverns;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import org.dyn4j.geometry.Geometry;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class CrystalOnField extends GamePieceOnFieldSimulation {
    public static final GamePieceInfo CRYSTAL_INFO = new GamePieceInfo(
            "Crystal", Geometry.createCircle(Units.inchesToMeters(11) / 2), Inches.of(2), Kilograms.of(0.2), 3.5, 5, 0.3);

    public CrystalOnField(Translation2d initialPosition) {
        super(CRYSTAL_INFO, new Pose2d(initialPosition, Rotation2d.kZero));
    }
}
