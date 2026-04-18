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
    assertEquals(1.0, v.getX(), 1e-9);
    assertEquals(0.0, v.getY(), 1e-9);
    assertEquals(1.0, v.getNorm(), 1e-9);
  }

  @Test
  void fullDiagonalPreservesUnitMagnitude() {
    double s = 1.0 / Math.sqrt(2.0);
    Translation2d v = JoystickDriveUtil.getLinearVelocityFromJoysticks(s, s, DEADBAND);
    assertEquals(1.0, v.getNorm(), 1e-6);
    assertEquals(45.0, Math.toDegrees(Math.atan2(v.getY(), v.getX())), 1e-6);
  }

  // Anti-snap: output direction must equal input direction for every angle.
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

  // Regression: per-axis sin^2 (the bug we fixed) snaps toward the dominant axis.
  // This test documents that, and proves the polar util does not.
  @Test
  void perAxisSinSquaredDemonstratesAxisSnap() {
    double mag = 0.9;
    double rad = Math.toRadians(30);
    double x = mag * Math.cos(rad);
    double y = mag * Math.sin(rad);

    double naiveX = Math.copySign(Math.pow(Math.sin(Math.PI / 2.0 * Math.abs(x)), 2), x);
    double naiveY = Math.copySign(Math.pow(Math.sin(Math.PI / 2.0 * Math.abs(y)), 2), y);
    double naiveDeg = Math.toDegrees(Math.atan2(naiveY, naiveX));

    Translation2d v = JoystickDriveUtil.getLinearVelocityFromJoysticks(x, y, DEADBAND);
    double polarDeg = Math.toDegrees(Math.atan2(v.getY(), v.getX()));

    assertTrue(naiveDeg < 27.0, "per-axis shaping should snap toward x-axis, got " + naiveDeg);
    assertEquals(30.0, polarDeg, 1e-6);
  }

  @Test
  void shapedMagnitudeIsMonotonicAndBounded() {
    double prev = -1.0;
    for (double m = 0.0; m <= 1.0 + 1e-9; m += 0.05) {
      double shaped = JoystickDriveUtil.shapeMagnitude(m);
      assertTrue(shaped >= 0.0 && shaped <= 1.0, "shape out of [0,1] at " + m);
      assertTrue(shaped >= prev - 1e-9, "not monotonic at " + m);
      prev = shaped;
    }
    assertEquals(0.0, JoystickDriveUtil.shapeMagnitude(0.0), EPS);
    assertEquals(1.0, JoystickDriveUtil.shapeMagnitude(1.0), 1e-9);
  }

  @Test
  void shapedMagnitudeIsFlatAtEndpoints() {
    // sin^2(pi/2 * x) has zero slope at x=0 and x=1 -- easy hold at rest and at max.
    double near0 = JoystickDriveUtil.shapeMagnitude(0.01);
    double near1Gap = 1.0 - JoystickDriveUtil.shapeMagnitude(0.99);
    assertTrue(near0 < 0.01, "expected gentle takeoff near zero, got " + near0);
    assertTrue(near1Gap < 0.01, "expected flat top near one, gap = " + near1Gap);
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
    assertTrue(pos > 0);
    assertTrue(neg < 0);
    assertEquals(pos, -neg, EPS);
  }

  @Test
  void omegaAtFullStickIsUnit() {
    assertEquals(1.0, JoystickDriveUtil.getOmegaFromJoysticks(1.0, DEADBAND), EPS);
    assertEquals(-1.0, JoystickDriveUtil.getOmegaFromJoysticks(-1.0, DEADBAND), EPS);
  }
}
