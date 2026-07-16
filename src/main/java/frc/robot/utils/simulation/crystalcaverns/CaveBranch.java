package frc.robot.utils.simulation.crystalcaverns;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.Goal;

/**
 *
 *
 * <h2>Simulates a branch on the field.</h2>
 *
 * <p>This class simulates a branch where crystals can be scored. This class should not be used directly and
 * instead should be used via a {@link CaveSim} which will handle an entire cave.
 */
public class CaveBranch extends Goal {
    public final int level; // 0 or 1
    public final int column; // 0 to 7

    private static Angle angleTolerance = Degrees.of(99999); // temp 
    private static final List<CaveBranch> allInstances = new ArrayList<>();

    /**
     *
     *
     * <h2>Sets the Angle Tolerance for All cave Branches</h2>
     *
     * <p>This configures the tolerance used for rotation validation across all cave branch instances. Changes take
     * effect immediately for all instances, including those already created.
     *
     * @param tolerance The angle tolerance to use for all branches
     */
    public static void setAngleTolerance(Angle tolerance) {
        angleTolerance = tolerance;
        // Update all existing instances
        for (CaveBranch branch : allInstances) branch.updateRotationChecker();
    }

    /**
     *
     *
     * <h2>Returns the required pose of a cave branch at the designated position.</h2>
     *
     * @param isBlue Wether the position is on the blue cave or the red cave.
     * @param level The level of the cave (0 indexed). Range of 0-3.
     * @param col The pole or Colum of the cave (0 indexed). Range of 0-11.
     * @return The pose of a cave branch with the specified stats.
     */
    public static Translation3d getPoseOfBranchAt(boolean isBlue, int level, int col) {
        Translation2d temp;
        double h;
        if (level == 0) {
            temp = isBlue ? FieldConstants.BLUE_LOWER_BRANCHES[col] : FieldConstants.RED_LOWER_BRANCHES[col];
            h = FieldConstants.LOWER_BRANCH_HEIGHT;
        } else {
            temp = isBlue ? FieldConstants.BLUE_UPPER_BRANCHES[col] : FieldConstants.RED_UPPER_BRANCHES[col];
            h = FieldConstants.UPPER_BRANCH_HEIGHT;
        }
        return new Translation3d(temp.getX(), temp.getY(), h);
    }

    private static final int MIN_LEVEL = 0;
    private static final int MAX_LEVEL = 1;

    /**
     *
     *
     * <h2>Creates a singular cave branch at the specified location </h2>
     *
     * @param arena The host arena of this cave.
     * @param isBlue Wether the position is on the blue cave or the red cave.
     * @param level The level of the cave (0 indexed). Range of 0-1.
     * @param column The pole or Colum of the cave (0 indexed). Range of 0-7.
     */
    public CaveBranch(CrystalCavernsArena arena, boolean isBlue, int level, int column) {
        super(
                arena,
                Inches.of(7),
                Inches.of(7),
                Inches.of(9.875),
                "Crystal",
                getPoseOfBranchAt(isBlue, level, column),
                isBlue,
                4,
                false);

        if (level < MIN_LEVEL || level > MAX_LEVEL) {
            throw new IllegalArgumentException(
                    "Invalid cave level: " + level + " (must be between " + MIN_LEVEL + " and " + MAX_LEVEL + ")");
        }

        this.level = level;
        this.column = column;

        // Set initial rotation checker
        updateRotationChecker();

        // Register this instance for future tolerance updates
        allInstances.add(this);
    }

    /**
     *
     *
     * <h2>Updates the Rotation Checker Based on Current Angle Tolerance</h2>
     *
     * <p>This method is called during construction and whenever the static angle tolerance is changed.
     */
    private void updateRotationChecker() {
        setNeededAngle(Rotation3d.kZero, angleTolerance);
    }

    /**
     *
     *
     * <h2>Gives the pose of the cave branch.</h2>
     *
     * @return This position of this branch as a pose3d.
     */
    public Pose3d getPose() {
        return new Pose3d(position, Rotation3d.kZero);
    }

    private static final int[] AUTO_POINTS_BY_LEVEL = {2, 3};
    private static final int[] TELEOP_POINTS_BY_LEVEL = {3, 4};

    @Override
    protected void addPoints() {
        System.out.println("Crystal scored on level: " + (level + 1) + " on the " + (isBlue ? "Blue " : "Red") + "cave");
        arena.addValueToMatchBreakdown(isBlue, "Auto/CrystalScoredInAuto", DriverStation.isAutonomous() ? 1 : 0);
        arena.addValueToMatchBreakdown(isBlue, "CrystalScoredOnLevel " + String.valueOf(level + 1), 1);

        int points = DriverStation.isAutonomous() ? AUTO_POINTS_BY_LEVEL[level] : TELEOP_POINTS_BY_LEVEL[level];
        arena.addToScore(isBlue, points);
    }

    @Override
    public void draw(List<Pose3d> drawList) {
        for (int i = 0; i < this.gamePieceCount; i++) {
            drawList.add(getPose().transformBy(new Transform3d(0, 0, Inches.of(2 * i + 1).in(Meters), Rotation3d.kZero)));
        }
    }
}