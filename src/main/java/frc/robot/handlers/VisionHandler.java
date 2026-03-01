package frc.robot.handlers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.handlers.StateSubsystem.State;
import frc.robot.subsystems.Vision;

public class VisionHandler extends SubsystemBase {

    public enum VisionState implements State {
        IDLE,
        CHASING
    }

    private Vision vision;
    private static VisionHandler instance;

    private VisionState desiredState = VisionState.IDLE;
    private VisionState currentState = VisionState.IDLE;

    private VisionHandler() {}

    public static VisionHandler getInstance() {
        if (instance == null) instance = new VisionHandler();
        return instance;
    }

    public void initialize(Vision vision) {
        this.vision = vision;
    }

    public void setDesiredState(VisionState state) {
        if (desiredState != state) {
            desiredState = state;
        }
    }

    public boolean isChasing() {
        return currentState == VisionState.CHASING;
    }

    public Vision getVision() {
        return vision;
    }

    @Override
    public void periodic() {
        currentState = desiredState;
        SmartDashboard.putString("VisionState", currentState.toString());
    }
}