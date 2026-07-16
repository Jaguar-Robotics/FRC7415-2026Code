package frc.robot.utils.simulation.crystalcaverns;

import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class GemstoneOnFly extends GamePieceProjectile {
    public GemstoneOnFly(
            Translation2d robotPosition,
            Translation2d shooterPositionOnRobot,
            ChassisSpeeds chassisSpeeds,
            Rotation2d shooterFacing,
            Distance initialHeight,
            LinearVelocity launchingSpeed,
            Angle shooterAngle) {
        super(
                GemstoneOnField.GEMSTONE_INFO,
                robotPosition,
                shooterPositionOnRobot,
                chassisSpeeds,
                shooterFacing,
                initialHeight,
                launchingSpeed,
                shooterAngle);

        // cone's origin is at bottom
        // might clip through ground if rotated...
        super.withTouchGroundHeight(Units.inchesToMeters(0.5)); 
        super.enableBecomesGamePieceOnFieldAfterTouchGround();
    }
}
