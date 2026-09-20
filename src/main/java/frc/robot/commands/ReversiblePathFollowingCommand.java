package frc.robot.commands;

import java.util.Collections;
import java.util.Optional;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A drop-in replacement for PathPlannerLib's {@code FollowPathCommand} that can be told, via a
 * {@link BooleanSupplier}, to drive backwards along the path it is following and then resume
 * forward progress from the exact point it stopped at (rather than restarting the path).
 *
 * <p>This works by tracking progress along the trajectory as an elapsed-time value that this
 * command advances or rewinds itself, instead of relying on a free-running {@link Timer} the way
 * {@code FollowPathCommand} does. While {@code reverseSupplier} is true, elapsed time counts
 * backwards, which causes the path following controller to chase a target point behind the
 * robot - i.e. drive it back the way it came. Once {@code reverseSupplier} goes false, elapsed
 * time counts forward again starting from wherever it was left, so the robot resumes the path
 * from that same point.
 */
public class ReversiblePathFollowingCommand extends Command {
  private final PathPlannerPath originalPath;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedsSupplier;
  private final BiConsumer<ChassisSpeeds, DriveFeedforwards> output;
  private final PathFollowingController controller;
  private final RobotConfig robotConfig;
  private final BooleanSupplier shouldFlipPath;
  private final BooleanSupplier reverseSupplier;
  private final EventScheduler eventScheduler;

  private PathPlannerPath path;
  private PathPlannerTrajectory trajectory;
  private double elapsedTime;
  private double lastTimestamp;
  private boolean reversing;

  /**
   * Construct a reversible path following command.
   *
   * @param path The path to follow
   * @param poseSupplier Function that supplies the current field-relative pose of the robot
   * @param speedsSupplier Function that supplies the current robot-relative chassis speeds
   * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards
   * @param controller Path following controller that will be used to follow the path
   * @param robotConfig The robot configuration
   * @param shouldFlipPath Should the path be flipped to the other side of the field?
   * @param reverseSupplier While this returns true, the robot drives backwards along the path
   *     instead of forwards. When it returns false again, the path resumes from wherever it was
   *     left off.
   * @param requirements Subsystems required by this command, usually just the drive subsystem
   */
  public ReversiblePathFollowingCommand(
      PathPlannerPath path,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      BooleanSupplier reverseSupplier,
      Subsystem... requirements) {
    this.originalPath = path;
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    this.output = output;
    this.controller = controller;
    this.robotConfig = robotConfig;
    this.shouldFlipPath = shouldFlipPath;
    this.reverseSupplier = reverseSupplier;
    this.eventScheduler = new EventScheduler();

    Set<Subsystem> driveRequirements = Set.of(requirements);
    addRequirements(requirements);

    var eventReqs = EventScheduler.getSchedulerRequirements(this.originalPath);
    if (!Collections.disjoint(driveRequirements, eventReqs)) {
      throw new IllegalArgumentException(
          "Events that are triggered during path following cannot require the drive subsystem");
    }
    addRequirements(eventReqs);

    this.path = this.originalPath;
    Optional<PathPlannerTrajectory> idealTrajectory = this.path.getIdealTrajectory(this.robotConfig);
    idealTrajectory.ifPresent(traj -> this.trajectory = traj);
  }

  @Override
  public void initialize() {
    if (shouldFlipPath.getAsBoolean() && !originalPath.preventFlipping) {
      path = originalPath.flipPath();
    } else {
      path = originalPath;
    }

    Pose2d currentPose = poseSupplier.get();
    ChassisSpeeds currentSpeeds = speedsSupplier.get();

    controller.reset(currentPose, currentSpeeds);

    double linearVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

    if (path.getIdealStartingState() != null) {
      boolean idealVelocity = Math.abs(linearVel - path.getIdealStartingState().velocityMPS()) <= 0.25;
      boolean idealRotation =
          !robotConfig.isHolonomic
              || Math.abs(
                      currentPose.getRotation().minus(path.getIdealStartingState().rotation()).getDegrees())
                  <= 30.0;
      if (idealVelocity && idealRotation) {
        trajectory = path.getIdealTrajectory(robotConfig).orElseThrow();
      } else {
        trajectory = path.generateTrajectory(currentSpeeds, currentPose.getRotation(), robotConfig);
      }
    } else {
      trajectory = path.generateTrajectory(currentSpeeds, currentPose.getRotation(), robotConfig);
    }

    PathPlannerAuto.setCurrentTrajectory(trajectory);
    PathPlannerAuto.currentPathName = originalPath.name;

    PathPlannerLogging.logActivePath(path);
    PPLibTelemetry.setCurrentPath(path);

    eventScheduler.initialize(trajectory);

    elapsedTime = 0.0;
    reversing = false;
    lastTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTimestamp;
    lastTimestamp = now;

    reversing = reverseSupplier.getAsBoolean();
    double totalTime = trajectory.getTotalTimeSeconds();
    if (reversing) {
      elapsedTime = Math.max(0.0, elapsedTime - dt);
    } else {
      elapsedTime = Math.min(totalTime, elapsedTime + dt);
    }

    var targetState = trajectory.sample(elapsedTime);
    if (!controller.isHolonomic() && path.isReversed()) {
      targetState = targetState.reverse();
    }

    Pose2d currentPose = poseSupplier.get();
    ChassisSpeeds currentSpeeds = speedsSupplier.get();

    ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);

    double currentVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

    PPLibTelemetry.setCurrentPose(currentPose);
    PathPlannerLogging.logCurrentPose(currentPose);

    PPLibTelemetry.setTargetPose(targetState.pose);
    PathPlannerLogging.logTargetPose(targetState.pose);

    PPLibTelemetry.setVelocities(
        currentVel,
        targetState.linearVelocity,
        currentSpeeds.omegaRadiansPerSecond,
        targetSpeeds.omegaRadiansPerSecond);

    output.accept(targetSpeeds, targetState.feedforwards);

    // Don't fire path events while backing up - only replay them going forward
    if (!reversing) {
      eventScheduler.execute(elapsedTime);
    }
  }

  @Override
  public boolean isFinished() {
    double totalTime = trajectory.getTotalTimeSeconds();
    return !reversing && (elapsedTime >= totalTime || !Double.isFinite(totalTime));
  }

  @Override
  public void end(boolean interrupted) {
    PathPlannerAuto.currentPathName = "";
    PathPlannerAuto.setCurrentTrajectory(null);

    if (!interrupted && path.getGoalEndState().velocityMPS() < 0.1) {
      output.accept(new ChassisSpeeds(), DriveFeedforwards.zeros(robotConfig.numModules));
    }

    PathPlannerLogging.logActivePath(null);

    eventScheduler.end();
  }
}
