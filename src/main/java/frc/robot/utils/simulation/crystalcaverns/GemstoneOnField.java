package frc.robot.utils.simulation.crystalcaverns;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.dyn4j.geometry.Geometry;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class GemstoneOnField extends GamePieceOnFieldSimulation {
    public static final GamePieceInfo GEMSTONE_INFO = new GamePieceInfo(
            "Gemstone", Geometry.createCircle(Units.inchesToMeters(4)), Inches.of(1), Pounds.of(1), 3.5, 5, 0.3);

    public GemstoneOnField(Translation2d initialPosition) {
        super(GEMSTONE_INFO, new Pose2d(initialPosition, new Rotation2d()));
    }
}
