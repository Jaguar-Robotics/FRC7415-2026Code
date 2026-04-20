// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class JoystickDriveUtil {

  private JoystickDriveUtil() {}

  // Preserves the team's existing sin²(pi/2 * mag) feel but applies it to the
  // magnitude rather than each axis. Per-axis shaping biases diagonals toward
  // the cardinal axes (axis snap).
  public static double shapeMagnitude(double magnitude) {
    double clamped = Math.min(Math.max(magnitude, 0.0), 1.0);
    double s = Math.sin((Math.PI / 2.0) * clamped);
    return s * s;
  }

  // Polar linear velocity: deadband on hypot, sin² on magnitude, preserve direction.
  // Output magnitude is in [0, 1]; caller multiplies by max speed.
  public static Translation2d getLinearVelocityFromJoysticks(double x, double y, double deadband) {
    double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), deadband);
    if (magnitude == 0.0) {
      return Translation2d.kZero;
    }
    Rotation2d direction = new Rotation2d(x, y);
    double shaped = shapeMagnitude(magnitude);
    return new Pose2d(Translation2d.kZero, direction)
        .transformBy(new Transform2d(shaped, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /*
  // Omega is 1D so a signed square preserves direction trivially.
  public static double getOmegaFromJoysticks(double omegaInput, double deadband) {
    double omega = MathUtil.applyDeadband(omegaInput, deadband);
    return omega * omega * Math.signum(omega);
  }
  */
  
public static double getOmegaFromJoysticks(double omegaInput, double deadband) {
    // 1. Initial Deadband Check (The 'Hard' Zero)
    if (Math.abs(omegaInput) < deadband) {
        return 0.0;
    }

    // 2. Linear Interpolation (The 'Mapping')
    // This maps [0.1 -> 1.0] joystick input to [10.0 -> 100.0] equation input.
    // This removes the "dead air" between the deadband and the curve.
    double absInput = Math.abs(omegaInput);
    double x = (absInput - deadband) / (1.0 - deadband) * (100.0 - 10.0) + 10.0;
    
    // 3. Apply the Equation: 100 * sin^2( (pi / 190) * (x - 5) )
    double term = (Math.PI / 190.0) * (x - 5.0);
    double shaped = 100.0 * Math.pow(Math.sin(term), 2);

    // 4. Return as a normalized multiplier (-1.0 to 1.0)
    return (shaped / 100.0) * Math.signum(omegaInput);
}
}
