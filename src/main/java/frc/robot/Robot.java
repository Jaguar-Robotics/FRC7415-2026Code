// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.HubShiftUtil;
import frc.robot.utils.HubShiftUtil.ShiftInfo;


public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        ShiftInfo official = HubShiftUtil.getOfficialShiftInfo();
        ShiftInfo shifted  = HubShiftUtil.getShiftedShiftInfo();

        // Official shift info 
        SmartDashboard.putString("Shift/Official/CurrentShift",   official.currentShift().toString());
        SmartDashboard.putString("Shift/Official/RemainingTime", String.format("%.1f", official.remainingTime()));
        SmartDashboard.putString("Shift/Official/ElapsedTime",   String.format("%.1f", official.elapsedTime()));
        SmartDashboard.putBoolean("Shift/Official/Active",        official.active());

        // Shifted shift info
        SmartDashboard.putString("Shift/Shifted/CurrentShift",   shifted.currentShift().toString());
        SmartDashboard.putNumber("Shift/Shifted/ElapsedTime",    shifted.elapsedTime());
        SmartDashboard.putNumber("Shift/Shifted/RemainingTime",  shifted.remainingTime());
        SmartDashboard.putBoolean("Shift/Shifted/Active",        shifted.active());


        
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        CameraServer.startAutomaticCapture();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

        if (isSimulation()) {
        RoboRioSim.setVInVoltage(12.5);
    }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
