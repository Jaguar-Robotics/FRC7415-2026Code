package frc.robot.handlers;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.BangBangShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;

public class ShooterHandler extends SubsystemBase implements StateSubsystem {


    public enum ShooterState implements State {
        SHOOTING,
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
        double DistM= drivetrain.getDistance();
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
        if (currentState != desiredState || currentState == ShooterState.SHOOTING) {
            handleStateChange();
        }
    }

    double DistMeters = 0;

    private void handleStateChange(){ 
        switch (desiredState) {
            case SHOOTING:
                shooter.setTargetVeloDistance(DistMeters); 
                break;
            case SLOW:
                shooter.setTargetVelocity(40);
                break;
            case FAST:
                shooter.setTargetVelocity(60);
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
    DistMeters = drivetrain.getDistance();
    update(); // Handle state transitions
    SmartDashboard.putString("ShooterState", currentState.toString());
}
}