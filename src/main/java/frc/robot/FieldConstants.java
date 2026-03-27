package frc.robot;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;

public class FieldConstants {

public static class Field{
    public static final double HalfwayY = Units.inchesToMeters(158.845);
}
public static class Tower {
    public String arielHasReturned = "I've returned";
    public static final double width = Units.inchesToMeters(49.25);
    public static final double depth = Units.inchesToMeters(45.0);
    public static final double height = Units.inchesToMeters(78.25);
    public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
    public static final double frontFaceX = Units.inchesToMeters(43.51);


  }
public static class Trench{
    public static final double RedTrenchX = Units.inchesToMeters(175);
    public static final double BlueTrenchX = Units.inchesToMeters(476.22);

    public static final double TopTrench = Units.inchesToMeters(291.47);
    public static final double BtmTrench = Units.inchesToMeters(26.22);
    
  }
}
