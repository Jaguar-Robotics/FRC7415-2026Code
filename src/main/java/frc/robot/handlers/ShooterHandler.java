package frc.robot.handlers;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.RobotContainer;

public class ShooterHandler extends SubsystemBase implements StateSubsystem {

    public  Joystick leftJoystick = new Joystick(2);


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
    private CommandSwerveDrivetrain drivetrain; // ← ADDED THIS
    private ShooterSubsystem shooter; // ← REMOVED "= new ShooterSubsystem()"

    private ShooterState desiredState = ShooterState.OFF;
    private ShooterState currentState = ShooterState.OFF;
    private Command shooterCommand = null; // ← ADD THIS


    private ShooterHandler() {}

    public static ShooterHandler getInstance() {
        if (instance == null) {
            instance = new ShooterHandler();
        }
        return instance;
    }

    public void initialize(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    }

    AngularVelocity TuneablefastShot = Constants.ShooterConstants.FastShot;
    public void adjustFastShot(double valu){
        TuneablefastShot = TuneablefastShot.plus(RPM.of(valu));

        if(currentState == ShooterState.TUNING){
            shooter.setVelocity(TuneablefastShot);
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
    if (currentState != desiredState) {
        System.out.println(" State changing from " + currentState + " to " + desiredState);
        handleStateChange(); // switch states
        }
    updateContinuousStates();
    }

    private void handleStateChange(){ 
        switch (desiredState) {
            case SHOOTING:
                //double DistMeters = drivetrain.GetFutureDistMeters();
                //shooter.setVelocityWithCalc(DistMeters).schedule(); 
                break;
            case SLOW:
                CommandScheduler.getInstance().schedule(shooter.setVelocity(Constants.ShooterConstants.SlowShot)); 
                break;
            case FAST:
                CommandScheduler.getInstance().schedule(shooter.setVelocity(Constants.ShooterConstants.FastShot)); 
                break;
            case TUNING:
                //shooter.setVelocity(TuneablefastShot).schedule();
                break;
            case OFF:
                CommandScheduler.getInstance().schedule(shooter.stop()); 
                break;
            default:
                CommandScheduler.getInstance().schedule(shooter.stop()); 
                break;
        }
        currentState = desiredState;
    }

    private void updateContinuousStates() {
        switch (currentState) {
            case SHOOTING:
                double DistMeters = drivetrain.GetFutureDistMeters();
                AngularVelocity targetSpeed = shooter.getCalcedRPM(DistMeters);
                CommandScheduler.getInstance().schedule(shooter.setVelocity(targetSpeed));
                break;
            
            case TUNING:
                CommandScheduler.getInstance().schedule(shooter.setVelocity(TuneablefastShot));
                break;
        
            default:
                break;
        }
    }
    public ShooterState getCurrentState() {
        return currentState;
    }

    @Override
public void periodic() {
    update(); // Handle state transitions
}
}