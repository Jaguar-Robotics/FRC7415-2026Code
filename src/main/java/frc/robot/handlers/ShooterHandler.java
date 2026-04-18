package frc.robot.handlers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.BangBangShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ShooterHandler extends SubsystemBase implements StateSubsystem {


    public enum ShooterState implements State {
        SHOOTING,
        SOTM,
        FAST,
        SLOW,
        REVERSE,
        OFF,
        TUNING,
    }

    private static ShooterHandler instance;
    // AFTER:
    private CommandSwerveDrivetrain drivetrain; 
    private BangBangShooterSubsystem shooter; 

    private ShooterState desiredState = ShooterState.OFF;
    private ShooterState currentState = ShooterState.OFF;

    Pose2d currPose2d;
    Pose2d LookaheadPose2d;

    private ShooterHandler() {}

    public static ShooterHandler getInstance() {
        if (instance == null) {
            instance = new ShooterHandler();
        }
        return instance;
    }

    public void initialize(CommandSwerveDrivetrain drivetrain, BangBangShooterSubsystem shooter) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    }

    double TuneablefastShot = 40;
    public void adjustFastShot(double valu){
        TuneablefastShot = TuneablefastShot + (valu);

        if(currentState == ShooterState.TUNING){
            shooter.setTargetVelocity(TuneablefastShot);
        }
    }

    public double getDistance(){
        if (drivetrain == null){
            return 0;
        }
        double DistM = drivetrain.getDistance(drivetrain.getPose());
        return DistM;
    }

    @Override
    public void setDesiredState(State state) {
        if (state instanceof ShooterState shooterState) {
            desiredState = shooterState;
        }
    }

    @Override
    public void handleStateTransition() {
        // Optional: delegate to update
        update();
    }

    @Override
    public void update() {
    if (shooter == null || drivetrain == null) { 
        System.out.println("ERROR: ShooterHandler not initialized! Call initialize() first.");
        return;
        }
        if (currentState != desiredState || currentState == ShooterState.SHOOTING || currentState == ShooterState.SOTM) {
            handleStateChange();
        }
    }

    double DistMeters = 0;
    double DistMetersLookAhead = 0;

    private void handleStateChange(){ 
        switch (desiredState) {
            case SHOOTING:
                //shooter.setTargetVeloDistance(currPose2d); 
                shooter.setTargetVeloDistance(DistMeters);
                break;
            case SOTM:
                //shooter.setTargetVeloDistance(LookaheadPose2d);
                shooter.setTargetVeloDistance(DistMetersLookAhead);
                break;
            case SLOW:
                shooter.setTargetVelocity(Constants.ShooterConstants.SlowShot);
                break;
            case FAST:
                shooter.setTargetVelocity(Constants.ShooterConstants.FastShot);
                break;
            case TUNING:
                shooter.setTargetVelocity(TuneablefastShot);
                break;
            case OFF:
                shooter.coast(); 
                break;
            default:
                //CommandScheduler.getInstance().schedule(shooter.stop()); 
                break;
        }
        currentState = desiredState;
    }

    public ShooterState getCurrentState() {
        return currentState;
    }

    @Override
public void periodic() {
    DistMeters = drivetrain.getDistance(drivetrain.getPose());
    DistMetersLookAhead = drivetrain.getLookaheadDistance() + 0.2;
    currPose2d = drivetrain.getPose();
    LookaheadPose2d = drivetrain.getLookaheadPose(); //0.2M is distance between orgin and shooter (its not acc but its what we measure from)
    update(); // Handle state transitions
    SmartDashboard.putString("ShooterState", currentState.toString());
}
}