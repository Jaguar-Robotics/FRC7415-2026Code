package frc.robot.utils.simulation.crystalcaverns;

import static frc.robot.utils.simulation.crystalcaverns.FieldConstants.*;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import org.ironmaple.simulation.SimulatedArena;

public class CrystalCavernsArena extends SimulatedArena {
    public static final class CrystalFieldObstaclesMap extends FieldMap {
        public CrystalFieldObstaclesMap() {
            super();

            super.addBorderLine(RED_MINE_CORNER_A, BLUE_MINE_CORNER_A);
            super.addBorderLine(BLUE_MINE_CORNER_A, BLUE_MINE_CORNER_B);
            super.addBorderLine(BLUE_MINE_CORNER_B, new Translation2d(FIELD_LENGTH, FIELD_WIDTH));
            super.addBorderLine(new Translation2d(FIELD_LENGTH, FIELD_WIDTH), new Translation2d(0, FIELD_WIDTH));
            super.addBorderLine(new Translation2d(0, FIELD_WIDTH), RED_MINE_CORNER_B);
            super.addBorderLine(RED_MINE_CORNER_B, RED_MINE_CORNER_A);

            // caves
            for (int i = 0; i < 8; i++) {
                super.addBorderLine(BLUE_CAVE_VORTICES[i], (BLUE_CAVE_VORTICES[(i + 1) % 8]));
                super.addBorderLine(RED_CAVE_VORTICES[i], (RED_CAVE_VORTICES[(i + 1) % 8]));
            }

            // cave exit trusses  
            for (int i = 0; i < 4; i++) {
                super.addBorderLine(BLUE_CAVE_EXIT_TRUSS_1_CORNERS[i], (BLUE_CAVE_EXIT_TRUSS_1_CORNERS[(i + 1) % 4]));
                super.addBorderLine(BLUE_CAVE_EXIT_TRUSS_2_CORNERS[i], (BLUE_CAVE_EXIT_TRUSS_2_CORNERS[(i + 1) % 4]));
                super.addBorderLine(RED_CAVE_EXIT_TRUSS_1_CORNERS[i], (RED_CAVE_EXIT_TRUSS_1_CORNERS[(i + 1) % 4]));
                super.addBorderLine(RED_CAVE_EXIT_TRUSS_2_CORNERS[i], (RED_CAVE_EXIT_TRUSS_2_CORNERS[(i + 1) % 4]));
            }

            // jewelry
            for (int i = 0; i < 6; i++) {
                super.addBorderLine(JEWELRY_VORTICES[i], (JEWELRY_VORTICES[(i + 1) % 6]));
            }
        }
    }

    public final CaveSim blueCaveSim;
    public final CaveSim redCaveSim;
    private final GemstoneHandler gemstoneHandler;

    public CrystalCavernsArena() {
        super(new CrystalFieldObstaclesMap());

        blueCaveSim = new CaveSim(this, true);
        redCaveSim = new CaveSim(this, false);
        gemstoneHandler = GemstoneHandler.getInstance();

        super.addCustomSimulation(blueCaveSim);
        super.addCustomSimulation(redCaveSim);
    }

    @Override
    protected void simulationSubTick(int subTickNum) {
        super.simulationSubTick(subTickNum);
        gemstoneHandler.periodic();
    }

    @Override
    public void placeGamePiecesOnField() {
        for (Pose2d notePosition : CRYSTAL_STARTING_POSITIONS)
            super.addGamePiece(new CrystalOnField(notePosition.getTranslation()));
    }

    @Override
    public synchronized List<Pose3d> getGamePiecesPosesByType(String type) {
        List<Pose3d> poses = super.getGamePiecesPosesByType(type);

        if (type.equals("Crystal")) {
            redCaveSim.draw(poses);
            blueCaveSim.draw(poses);
        } else if (type.equals("Gemstone")) {
            gemstoneHandler.draw(poses);
        }

        return poses;
    }

    @Override
    public synchronized void clearGamePieces() {
        super.clearGamePieces();
        redCaveSim.clearCave();
        blueCaveSim.clearCave();
        gemstoneHandler.reset();
    }
}
