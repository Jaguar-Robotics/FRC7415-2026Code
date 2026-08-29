package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
  
  private final CommandSwerveDrivetrain drivetrain;
  private boolean useMegaTag2 = false; // Set to false to use MegaTag1 SWITCHING BECAUSE MT1 more reliable somehow
  
  /** Creates a new Vision subsystem */
  public Vision(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  /**
   * Updates pose estimator with Limelight vision measurements
   * Called automatically in periodic().
   */

    // All Pose estimating limelights names
  private final String[] PosLimelights = {
    "limelight-fl",
    "limelight-fr"
    // add more as needed
  };

  public void updateVisionMeasurements(String LLName) {
    boolean doRejectUpdate = false;
    
    if (!useMegaTag2) {
      // MegaTag1 mode
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(LLName);

      if (mt1 == null){
        return;
      }
      
      // Reject if single tag with high ambiguity
      if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
        if (mt1.rawFiducials[0].ambiguity > 0.7) {
          doRejectUpdate = true;
        }
        if (mt1.rawFiducials[0].distToCamera > 3) {
          doRejectUpdate = true;
        }
      }
      
      // Reject if no tags seen
      if (mt1.tagCount == 0) {
        doRejectUpdate = true;
      }
      
      if (!doRejectUpdate) {
        // Add vision measurement to drivetrain
        drivetrain.addVisionMeasurement(
          mt1.pose,
          mt1.timestampSeconds,
          VecBuilder.fill(0.5, 0.5, 0.9)
        );
      }
    } else {
      // MegaTag2 mode
      // Set robot orientation for MegaTag2
      LimelightHelpers.SetRobotOrientation(
        LLName,
        drivetrain.getState().Pose.getRotation().getDegrees(),
        0, 0, 0, 0, 0
      );
      
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LLName);
      
      if (mt2 == null){
        return;
      }
      
      // Reject if spinning too fast (> 720 deg/s)
      if (Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond) > Math.toRadians(720)) {
        doRejectUpdate = true;
      }
      
      // Reject if no tags seen
      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      }
      
      if (!doRejectUpdate) {
        // Add vision measurement to drivetrain
        drivetrain.addVisionMeasurement(
          mt2.pose,
          mt2.timestampSeconds,
          VecBuilder.fill(0.7, 0.7, 0.9)  //CHAT TOLD ME TO CHANGE (i unchanged it back)
        ); 
      }
    }
  } 

  public Pose2d getSingleTagPose(int Tag, String limelightName) {
      LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, new int[]{Tag});

      if (!LimelightHelpers.getTV(limelightName)) {
          return null;
      }

      Pose3d pose3d = LimelightHelpers.getTargetPose3d_RobotSpace(limelightName);

      if (pose3d == null) {
          return null;
      }

    return drivetrain.getPose().transformBy(
        new Transform2d(pose3d.toPose2d().getTranslation(), pose3d.toPose2d().getRotation())
    );

    }

  
  public Pose2d SingleTagpose = new Pose2d(5,5,(Rotation2d.kZero));
  @Override
  public void periodic() {
    for (String limeLight : PosLimelights) {
      if (RobotContainer.ShowCaseMode) {
        SingleTagpose = getSingleTagPose(13, limeLight);
      return;
      }
      else {
         updateVisionMeasurements(limeLight); 
      }       
    }
  }
}