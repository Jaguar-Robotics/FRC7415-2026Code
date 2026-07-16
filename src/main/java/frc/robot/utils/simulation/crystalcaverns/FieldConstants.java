package frc.robot.utils.simulation.crystalcaverns;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static final double FIELD_LENGTH = Units.inchesToMeters(648);
    public static final double FIELD_WIDTH = Units.inchesToMeters(324);

    public static final Pose2d CENTER = new Pose2d(FIELD_LENGTH / 2.0, FieldConstants.FIELD_WIDTH / 2.0,
            Rotation2d.kZero);
    public static final Translation2d CENTER_ORIGIN = CENTER.getTranslation();

    // crystal caverns
    public static final Pose2d FIRST_RED_CRYSTAL_STARTING_POSE = CENTER
            .transformBy(new Transform2d(6.669830, -2.771845, Rotation2d.kZero));
    public static final Pose2d FIRST_BLUE_CRYSTAL_STARTING_POSE = CENTER
            .transformBy(new Transform2d(-6.669830, -2.771845, Rotation2d.kZero));

    public static final Pose2d FIRST_CAVE_CRYSTAL_STARTING_POSE = CENTER
            .transformBy(new Transform2d(0, -0.762000, Rotation2d.kZero));
    public static final Pose2d FIRST_HP_CRYSTAL_STARTING_POSE = CENTER
            .transformBy(new Transform2d(0, 0.762000, Rotation2d.kZero));

    public static final double FAR_CRYSTAL_STAGING_SEPARATION = 1.604022; // m
    public static final double CENTER_CRYSTAL_STAGING_SEPARATION = 0.889000; // m

    public static final Pose2d[] CRYSTAL_STARTING_POSITIONS = new Pose2d[16];
    static {
        for (int i = 0; i < 4; i++) {
            CRYSTAL_STARTING_POSITIONS[i] = FIRST_RED_CRYSTAL_STARTING_POSE
                    .transformBy(new Transform2d(0, FAR_CRYSTAL_STAGING_SEPARATION * i, Rotation2d.kZero));
        }
        for (int i = 0; i < 4; i++) {
            CRYSTAL_STARTING_POSITIONS[i + 4] = FIRST_BLUE_CRYSTAL_STARTING_POSE
                    .transformBy(new Transform2d(0, FAR_CRYSTAL_STAGING_SEPARATION * i, Rotation2d.kZero));
        }
        for (int i = 0; i < 4; i++) {
            CRYSTAL_STARTING_POSITIONS[i + 8] = FIRST_CAVE_CRYSTAL_STARTING_POSE
                    .transformBy(new Transform2d(0, CENTER_CRYSTAL_STAGING_SEPARATION * -i, Rotation2d.kZero));
        }
        for (int i = 0; i < 4; i++) {
            CRYSTAL_STARTING_POSITIONS[i + 12] = FIRST_HP_CRYSTAL_STARTING_POSE
                    .transformBy(new Transform2d(0, CENTER_CRYSTAL_STAGING_SEPARATION * i, Rotation2d.kZero));
        }
    }

    public static final Pose2d BLUE_CAVE_CENTER = CENTER.transformBy(new Transform2d(-4.129830, 0, Rotation2d.kZero));
    public static final Translation2d BLUE_CAVE_ORIGIN = BLUE_CAVE_CENTER.getTranslation();

    public static final Translation2d[] BLUE_CAVE_VORTICES = new Translation2d[8];
    static {
        for (int i = 0; i < BLUE_CAVE_VORTICES.length; i++) {
            BLUE_CAVE_VORTICES[i] = new Translation2d(1.170789, Rotation2d.fromDegrees(45 * i)).plus(BLUE_CAVE_ORIGIN);
        }
    }

    public static final Translation2d[] BLUE_LOWER_BRANCHES = new Translation2d[] {
            new Translation2d(0.762663, 0.315905).plus(BLUE_CAVE_ORIGIN),
            new Translation2d(0.315905, 0.762663).plus(BLUE_CAVE_ORIGIN),
            new Translation2d(-0.315905, 0.762663).plus(BLUE_CAVE_ORIGIN), // classifier side
            new Translation2d(-0.762663, 0.315905).plus(BLUE_CAVE_ORIGIN),
            new Translation2d(-0.762663, -0.315905).plus(BLUE_CAVE_ORIGIN), // HP side
            new Translation2d(-0.315905, -0.762663).plus(BLUE_CAVE_ORIGIN),
            new Translation2d(0.315905, -0.762663).plus(BLUE_CAVE_ORIGIN), // scoring table side
            new Translation2d(0.762663, -0.315905).plus(BLUE_CAVE_ORIGIN)
    };

    public static final Translation2d[] BLUE_UPPER_BRANCHES = new Translation2d[] {
            new Translation2d(0.218704, 0.218704).plus(BLUE_CAVE_ORIGIN),
            new Translation2d(-0.218704, 0.218704).plus(BLUE_CAVE_ORIGIN), // classifier side
            new Translation2d(-0.218704, -0.218704).plus(BLUE_CAVE_ORIGIN), // HP side
            new Translation2d(0.218704, -0.218704).plus(BLUE_CAVE_ORIGIN) // scoring table side
    };

    public static final Translation2d[] RED_CAVE_VORTICES = Arrays.stream(BLUE_CAVE_VORTICES)
            .map(pointAtBlue -> new Translation2d(FIELD_LENGTH - pointAtBlue.getX(), pointAtBlue.getY()))
            .toArray(Translation2d[]::new);
    public static final Translation2d[] RED_LOWER_BRANCHES = Arrays.stream(BLUE_LOWER_BRANCHES)
            .map(pointAtBlue -> new Translation2d(FIELD_LENGTH - pointAtBlue.getX(), pointAtBlue.getY()))
            .toArray(Translation2d[]::new);
    public static final Translation2d[] RED_UPPER_BRANCHES = Arrays.stream(BLUE_UPPER_BRANCHES)
            .map(pointAtBlue -> new Translation2d(FIELD_LENGTH - pointAtBlue.getX(), pointAtBlue.getY()))
            .toArray(Translation2d[]::new);

    public static final double LOWER_BRANCH_HEIGHT = Units.inchesToMeters(19);
    public static final double UPPER_BRANCH_HEIGHT = Units.inchesToMeters(32);

    public static final Pose3d BLUE_GEMSTONE_STARTING_POSE = new Pose3d(
        BLUE_CAVE_CENTER.getX(), BLUE_CAVE_CENTER.getY(), Units.inchesToMeters(32.813), Rotation3d.kZero);
    public static final Pose3d RED_GEMSTONE_STARTING_POSE = new Pose3d(
        FIELD_LENGTH - BLUE_CAVE_CENTER.getX(), BLUE_CAVE_CENTER.getY(), Units.inchesToMeters(32.813), Rotation3d.kZero);
    
    public static final Translation2d[] JEWELRY_VORTICES = new Translation2d[6];
    static {
        for (int i = 0; i < JEWELRY_VORTICES.length; i++) {
            JEWELRY_VORTICES[i] = new Translation2d(0.293294, Rotation2d.fromDegrees(60 * i)).plus(CENTER_ORIGIN);
        }
    }

    public static final double JEWELRY_POS_TOLERANCE = Units.inchesToMeters(12);
    public static final double JEWELRY_ROT_TOLERANCE = Units.degreesToRadians(30);
    public static final double JEWELRY_MIN_Z = Units.inchesToMeters(12);

    public static final Translation2d RED_MINE_CORNER_A = new Translation2d(FIELD_LENGTH / 2.0 - 6.493848, 0);
    public static final Translation2d BLUE_MINE_CORNER_A = new Translation2d(FIELD_LENGTH / 2.0 + 6.493848, 0);
    public static final Translation2d RED_MINE_CORNER_B = new Translation2d(0, FIELD_WIDTH / 2.0 - 3.044640);
    public static final Translation2d BLUE_MINE_CORNER_B = new Translation2d(
            FIELD_LENGTH, FIELD_WIDTH / 2.0 - 3.044640);

    public static final List<Pose2d> MINES = new ArrayList<Pose2d>();
    static {
        MINES.add(new Pose2d(
            (BLUE_MINE_CORNER_A.getX() + BLUE_MINE_CORNER_B.getX()) / 2.0,
            (BLUE_MINE_CORNER_A.getY() + BLUE_MINE_CORNER_B.getY()) / 2.0,
            Rotation2d.fromDegrees(120)));
        MINES.add(new Pose2d(
            (RED_MINE_CORNER_A.getX() + RED_MINE_CORNER_B.getX()) / 2.0,
            (RED_MINE_CORNER_A.getY() + RED_MINE_CORNER_B.getY()) / 2.0,
            Rotation2d.fromDegrees(60)));
    }

    public static final Translation2d[] BLUE_CAVE_EXIT_TRUSS_1_CORNERS = new Translation2d[] {
            new Translation2d(-3.169557, 2.693090).plus(CENTER_ORIGIN),
            new Translation2d(-3.483882, 2.693090).plus(CENTER_ORIGIN),
            new Translation2d(-3.483882, 2.997890).plus(CENTER_ORIGIN),
            new Translation2d(-3.169557, 2.997890).plus(CENTER_ORIGIN),
    };
    public static final Translation2d[] BLUE_CAVE_EXIT_TRUSS_2_CORNERS = new Translation2d[] {
            new Translation2d(-6.531882, 2.693090).plus(CENTER_ORIGIN),
            new Translation2d(-6.846207, 2.693090).plus(CENTER_ORIGIN),
            new Translation2d(-6.846207, 2.997890).plus(CENTER_ORIGIN),
            new Translation2d(-6.531882, 2.997890).plus(CENTER_ORIGIN),
    };
    public static final Translation2d[] RED_CAVE_EXIT_TRUSS_1_CORNERS = Arrays.stream(BLUE_CAVE_EXIT_TRUSS_1_CORNERS)
            .map(pointAtBlue -> new Translation2d(FIELD_LENGTH - pointAtBlue.getX(), pointAtBlue.getY()))
            .toArray(Translation2d[]::new);
    public static final Translation2d[] RED_CAVE_EXIT_TRUSS_2_CORNERS = Arrays.stream(BLUE_CAVE_EXIT_TRUSS_2_CORNERS)
            .map(pointAtBlue -> new Translation2d(FIELD_LENGTH - pointAtBlue.getX(), pointAtBlue.getY()))
            .toArray(Translation2d[]::new);

    public static final List<Pose2d> CAVE_ALIGN_POSES = new ArrayList<>();
    public static final double BRANCH_TO_ROBOT = Units.inchesToMeters(30);
    static {
        for (int i = 0; i < BLUE_LOWER_BRANCHES.length; i++) {
            Rotation2d blueRot = Rotation2d.fromDegrees(45 * i + 22.5);
            CAVE_ALIGN_POSES.add(new Pose2d(BLUE_LOWER_BRANCHES[i].plus(new Translation2d(BRANCH_TO_ROBOT, blueRot)), blueRot));
            
            Rotation2d redRot = Rotation2d.fromDegrees(-45 * i + 157.5);
            CAVE_ALIGN_POSES.add(new Pose2d(RED_LOWER_BRANCHES[i].plus(new Translation2d(BRANCH_TO_ROBOT, redRot)), redRot));
        }
    }

    public static final List<Pose2d> CLASSIFIER_ALIGN_POSES = new ArrayList<>();
    static {
        CLASSIFIER_ALIGN_POSES.add(new Pose2d(15.955, 5.2, Rotation2d.kZero));
        CLASSIFIER_ALIGN_POSES.add(new Pose2d(15.955, 5.9, Rotation2d.kZero));
        CLASSIFIER_ALIGN_POSES.add(new Pose2d(FIELD_LENGTH - 15.955, 5.2, Rotation2d.k180deg));
        CLASSIFIER_ALIGN_POSES.add(new Pose2d(FIELD_LENGTH - 15.955, 5.9, Rotation2d.k180deg));
    }

    public static final List<Pose2d> JEWELRY_ALIGN_POSES = new ArrayList<>();
    static {
        for (int i = 0; i < 6; i++) {
            Rotation2d rot = Rotation2d.fromDegrees(60 * i + 30);
            JEWELRY_ALIGN_POSES.add(new Pose2d(CENTER_ORIGIN.plus(new Translation2d(BRANCH_TO_ROBOT, rot)), rot.plus(Rotation2d.k180deg)));
        }
    }

    // hero heist constants
    public static final double LOW_FOOTHILL_HEIGHT = Units.inchesToMeters(46.5);
    public static final double HIGH_FOOTHILL_HEIGHT = Units.inchesToMeters(103.5);
    public static final double UPTOWN_HEIGHT = Units.inchesToMeters(68.574);
    public static final double DOWNTOWN_HEIGHT = Units.inchesToMeters(42 + 6); // shoot above opening

    public static final Pose2d LEFT_UPTOWN_DISTRICT = new Pose2d(Units.inchesToMeters(204),
            Units.inchesToMeters(325.882), Rotation2d.kZero);
    public static final Pose2d LEFT_DOWNTOWN_DISTRICT = new Pose2d(Units.inchesToMeters(204), Units.inchesToMeters(-1),
            Rotation2d.kZero);
    public static final double DISTRICT_SEPARATION = Units.inchesToMeters(48);
    public static final int NUM_DISTRICTS = 6; // 6 uptown and 6 downtown districts

    public static final List<Pose2d> FOOTHILL_DISTRICTS = new ArrayList<>();
    public static final List<Pose2d> UPTOWN_DISTRICTS = new ArrayList<>();
    public static final List<Pose2d> DOWNTOWN_DISTRICTS = new ArrayList<>();

    static {
        // east foothills, red protected zone
        FOOTHILL_DISTRICTS.add(new Pose2d(0.4127, 7.196, Rotation2d.kZero));
        FOOTHILL_DISTRICTS.add(new Pose2d(1.223, 7.882, Rotation2d.kZero));

        // west foothills, blue protected zone
        FOOTHILL_DISTRICTS.add(new Pose2d(FIELD_LENGTH - 0.4127, 7.196, Rotation2d.kZero));
        FOOTHILL_DISTRICTS.add(new Pose2d(FIELD_LENGTH - 1.223, 7.882, Rotation2d.kZero));

        for (int i = 0; i < NUM_DISTRICTS; i++) {
            UPTOWN_DISTRICTS.add(
                    LEFT_UPTOWN_DISTRICT.plus(new Transform2d(i * DISTRICT_SEPARATION, 0, Rotation2d.kZero)));

            DOWNTOWN_DISTRICTS.add(
                    LEFT_DOWNTOWN_DISTRICT.plus(new Transform2d(i * DISTRICT_SEPARATION, 0, Rotation2d.kZero)));
        }
    }

    public static final int BLUE_BUBBLE_ROWS = 5;
    public static final int RED_BUBBLE_ROWS = 5;
    public static final int BLUE_BUBBLE_COLS = 3;
    public static final int RED_BUBBLE_COLS = 3;
    public static final double BUBBLE_ROW_SEPARATION = Units.inchesToMeters(24);
    public static final double BUBBLE_COL_SEPARATION = Units.inchesToMeters(48);
    public static final Pose2d[] BLUE_BUBBLE_STARTING_POSITIONS = new Pose2d[BLUE_BUBBLE_ROWS * BLUE_BUBBLE_COLS];
    public static final Pose2d[] RED_BUBBLE_STARTING_POSITIONS = new Pose2d[RED_BUBBLE_ROWS * BLUE_BUBBLE_COLS];
    public static final Pose2d FIRST_BLUE_BUBBLE_STARTING_POSE = new Pose2d(Units.inchesToMeters(204),
            Units.inchesToMeters(114), Rotation2d.kZero);
    public static final Pose2d FIRST_RED_BUBBLE_STARTING_POSE = new Pose2d(Units.inchesToMeters(444),
            Units.inchesToMeters(114), Rotation2d.kZero);

    static {
        int index = 0;
        for (int row = 0; row < BLUE_BUBBLE_ROWS; row++) {
            for (int col = 0; col < BLUE_BUBBLE_COLS; col++) {
                BLUE_BUBBLE_STARTING_POSITIONS[index++] = FIRST_BLUE_BUBBLE_STARTING_POSE.transformBy(new Transform2d(
                        col * BUBBLE_COL_SEPARATION, row * BUBBLE_ROW_SEPARATION, Rotation2d.kZero));
            }
        }
        index = 0;
        for (int row = 0; row < RED_BUBBLE_ROWS; row++) {
            for (int col = 0; col < RED_BUBBLE_COLS; col++) {
                RED_BUBBLE_STARTING_POSITIONS[index++] = FIRST_RED_BUBBLE_STARTING_POSE.transformBy(new Transform2d(
                        col * -BUBBLE_COL_SEPARATION, row * BUBBLE_ROW_SEPARATION, Rotation2d.kZero));
            }
        }
    }

    // reefscape constants, old
    public static final int[] BLUE_REEF_TAG_IDS = { 18, 19, 20, 21, 22, 17 };
    public static final int[] BLUE_CORAL_STATION_TAG_IDS = { 12, 13 };
    public static final int[] RED_REEF_TAG_IDS = { 7, 6, 11, 10, 9, 8 };
    public static final int[] RED_CORAL_STATION_TAG_IDS = { 1, 2 };
    public static final int[] ALL_REEF_TAG_IDS = { 18, 19, 20, 21, 22, 17, 7, 6, 11, 10, 9, 8 };

    public static final List<Pose2d> CORAL_STATIONS = new ArrayList<>();


    public static final Pose3d[] REEF_TAG_POSES = new Pose3d[RED_REEF_TAG_IDS.length + BLUE_REEF_TAG_IDS.length];
    public static final List<Pose2d> REEF_TAGS = new ArrayList<>();

    static {
        for (Pose3d tag : REEF_TAG_POSES) {
            REEF_TAGS.add(tag.toPose2d());
        }
    }

    public static final Transform3d HIGH_ALGAE_TRANSFORM = new Transform3d(Units.inchesToMeters(-6), 0,
            Units.inchesToMeters(39.575), Rotation3d.kZero);
    public static final Transform3d LOW_ALGAE_TRANSFORM = new Transform3d(Units.inchesToMeters(-6), 0,
            Units.inchesToMeters(23.675), Rotation3d.kZero);

    public static final Pose3d[] REEF_ALGAE_POSES = new Pose3d[REEF_TAG_POSES.length];

    static {
        for (int i = 0; i < REEF_ALGAE_POSES.length; i++) {
            REEF_ALGAE_POSES[i] = REEF_TAG_POSES[i].plus(i % 2 == 0 ? HIGH_ALGAE_TRANSFORM : LOW_ALGAE_TRANSFORM);
        }
    }

    public static final double BARGE_X = FIELD_LENGTH / 2.0;
    public static final double BARGE_WIDTH = Units.inchesToMeters(40) / 2.0;
    public static final double BARGE_HEIGHT = Units.inchesToMeters(74 + 8);
    public static final double BARGE_HEIGHT_TOLERANCE = Units.inchesToMeters(12);

    public static final double TRANSLATIONAL_TOLERANCE = Units.inchesToMeters(16);
    public static final double DROP_COOLDOWN = 2.0;
}
