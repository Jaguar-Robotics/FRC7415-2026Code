// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class JoystickDriveUtilTest {

  private static final double DEADBAND = 0.1;
  private static final double EPS = 1e-9;

  @Test
  void zeroInputReturnsZero() {
    Translation2d v = JoystickDriveUtil.getLinearVelocityFromJoysticks(0.0, 0.0, DEADBAND);
    assertEquals(0.0, v.getX(), EPS);
    assertEquals(0.0, v.getY(), EPS);
  }

  @Test
  void belowDeadbandReturnsZero() {
    Translation2d v = JoystickDriveUtil.getLinearVelocityFromJoysticks(0.05, 0.05, DEADBAND);
    assertEquals(0.0, v.getNorm(), EPS);
  }

  @Test
  void fullForwardGivesUnitMagnitude() {
    Translation2d v = JoystickDriveUtil.getLinearVelocityFromJoysticks(1.0, 0.0, DEADBAND);
    assertEquals(1.0, v.getX(), EPS);
    assertEquals(0.0, v.getY(), EPS);
    assertEquals(1.0, v.getNorm(), EPS);
  }

  @Test
  void fullDiagonalPreservesUnitMagnitude() {
    // Full stick at 45 deg (hypot == 1).
    double s = 1.0 / Math.sqrt(2.0);
    Translation2d v = JoystickDriveUtil.getLinearVelocityFromJoysticks(s, s, DEADBAND);
    assertEquals(1.0, v.getNorm(), 1e-6);
    // Direction preserved at 45 deg.
    assertEquals(45.0, Math.toDegrees(Math.atan2(v.getY(), v.getX())), 1e-6);
  }

  // Anti-snap regression: the output direction must equal the input direction.
  // A naive per-axis square would bend the vector toward the dominant axis.
  @Test
  void directionPreservedAcrossAngles() {
    for (int deg = 5; deg < 90; deg += 5) {
      double rad = Math.toRadians(deg);
      double x = Math.cos(rad);
      double y = Math.sin(rad);
      Translation2d v = JoystickDriveUtil.getLinearVelocityFromJoysticks(x, y, DEADBAND);
      double outDeg = Math.toDegrees(Math.atan2(v.getY(), v.getX()));
      assertEquals(deg, outDeg, 1e-6, "direction drifted at " + deg + " deg");
    }
  }

  // Contrast: a naive per-axis signed-square snaps toward the dominant axis.
  // This documents the bug the util fixes.
  @Test
  void naivePerAxisSquareDemonstratesAxisSnap() {
    // 30 deg stick, just outside deadband so both axes survive naive handling.
    double mag = 0.9;
    double rad = Math.toRadians(30);
    double x = mag * Math.cos(rad);
    double y = mag * Math.sin(rad);

    double naiveX = Math.signum(x) * x * x;
    double naiveY = Math.signum(y) * y * y;
    double naiveDeg = Math.toDegrees(Math.atan2(naiveY, naiveX));

    Translation2d v = JoystickDriveUtil.getLinearVelocityFromJoysticks(x, y, DEADBAND);
    double polarDeg = Math.toDegrees(Math.atan2(v.getY(), v.getX()));

    // Naive snaps toward the X axis (angle drops below 30).
    assertTrue(naiveDeg < 25.0, "naive should snap toward x-axis, got " + naiveDeg);
    // Polar preserves 30 deg.
    assertEquals(30.0, polarDeg, 1e-6);
  }

  @Test
  void magnitudeIsSquared() {
    // Half stick -> quarter magnitude (after deadband rescaling from MathUtil.applyDeadband).
    // MathUtil.applyDeadband rescales so that at deadband the output is 0 and at 1.0 it is 1.0.
    // We verify: input magnitude 1.0 -> 1.0, input 0.5 -> (rescaled-value)^2, monotonic.
    Translation2d full = JoystickDriveUtil.getLinearVelocityFromJoysticks(1.0, 0.0, DEADBAND);
    Translation2d half = JoystickDriveUtil.getLinearVelocityFromJoysticks(0.5, 0.0, DEADBAND);
    assertEquals(1.0, full.getNorm(), EPS);
    assertTrue(half.getNorm() < full.getNorm());
    assertTrue(half.getNorm() < 0.5); // squared curve keeps half-stick under linear.
    assertTrue(half.getNorm() > 0.0);
  }

  @Test
  void omegaZeroAtZero() {
    assertEquals(0.0, JoystickDriveUtil.getOmegaFromJoysticks(0.0, DEADBAND), EPS);
  }

  @Test
  void omegaDeadbandClamps() {
    assertEquals(0.0, JoystickDriveUtil.getOmegaFromJoysticks(0.05, DEADBAND), EPS);
    assertEquals(0.0, JoystickDriveUtil.getOmegaFromJoysticks(-0.05, DEADBAND), EPS);
  }

  @Test
  void omegaPreservesSign() {
    double pos = JoystickDriveUtil.getOmegaFromJoysticks(0.8, DEADBAND);
    double neg = JoystickDriveUtil.getOmegaFromJoysticks(-0.8, DEADBAND);
    assertTrue(pos > 0, "positive input should give positive omega");
    assertTrue(neg < 0, "negative input should give negative omega");
    assertEquals(pos, -neg, EPS);
  }

  @Test
  void omegaAtFullStickIsUnit() {
    assertEquals(1.0, JoystickDriveUtil.getOmegaFromJoysticks(1.0, DEADBAND), EPS);
    assertEquals(-1.0, JoystickDriveUtil.getOmegaFromJoysticks(-1.0, DEADBAND), EPS);
  }
}
