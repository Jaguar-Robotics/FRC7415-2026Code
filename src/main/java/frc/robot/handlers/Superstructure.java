package frc.robot.handlers;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.handlers.IndexerHighHandler.IndexerHighState;
import frc.robot.handlers.IndexerLowHandler.IndexerLowState;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerHighSubsystem;
import frc.robot.subsystems.IndexerLowSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;

public class Superstructure extends SubsystemBase {

  // Define your states
  public enum SuperstructureState {
    STATIONARYSHOT,
    INTAKE,
    SLOWSHOT,
    REVERSE,
    SPINUP,
    TUNING,
    OFF,
    IDLE,
    SPINUPSLOW,
    SPINUPFAST,
    FASTSHOT,
    CLIMBPREP,
    CLIMBED
  }

  private static Superstructure instance;
  private ShooterSubsystem shooter; 
  private CommandSwerveDrivetrain drivetrain;
  private ClimbSubsystem climber;

  
  //private final ShooterSubsystem Shooter = new ShooterSubsystem();
  private final ShooterHandler shooterHandler = ShooterHandler.getInstance();

  //private final IntakeSubsystem Intake = new IntakeSubsystem();
  private final IntakeHandler intakeHandler = IntakeHandler.getInstance();

  //private final HopperSubsystem Hopper = new HopperSubsystem();
  private final HopperHandler hopperHandler = HopperHandler.getInstance();

  //private final IndexerSubsystem Indexer = new IndexerSubsystem();
  private final IndexerHighHandler indexerHighHandler = IndexerHighHandler.getInstance();
  private final IndexerLowHandler indexerLowHandler = IndexerLowHandler.getInstance();

  private final DriveHandler driveHandler = DriveHandler.getInstance();

  private final ClimbHandler climbHandler = ClimbHandler.getInstance();

  private SuperstructureState desiredState = SuperstructureState.IDLE;
  private SuperstructureState currentState = SuperstructureState.IDLE;
  private Angle targetAngle = Degrees.of(0);
  

  public static Superstructure getInstance(){
    if (instance == null) {
      instance = new Superstructure();
    }
    return instance;
  }

    /**
   * Initialize superstructure with required subsystems
   */
  public void initialize(ShooterSubsystem shooter, CommandSwerveDrivetrain drivetrain, ClimbSubsystem climber) {
      this.shooter = shooter;
      this.drivetrain = drivetrain;
      this.climber = climber;
  }

  private Superstructure() {}

  /** Call this from commands or joystick logic to set the next goal. */
  public void setDesiredState(SuperstructureState newState) {
    System.out.print("SetDesiredState:" + newState);
    if (desiredState != newState) {
      desiredState = newState;
      handleStateTransition();
    }
  }

  /** Handles transitions between states (2910-style logic). */
  private void handleStateTransition() {
    switch (desiredState) {
      case IDLE:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.TELEOPDRIVE);
        climbHandler.setDesiredState(ClimbHandler.ClimbState.OFF);
        break;
      case INTAKE:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.FASTINTAKE);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.SNAKE);
        climbHandler.setDesiredState(ClimbHandler.ClimbState.OFF);
        break;
      case SPINUP:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.SHOOTING);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.AUTOALLIGN);
        climbHandler.setDesiredState(ClimbHandler.ClimbState.OFF);
        break;
      case STATIONARYSHOT:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.SHOOTING);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.FASTINTAKE);
        hopperHandler.setDesiredState(HopperHandler.HopperState.FAST);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.SLOWINTAKE);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.SLOWINTAKE);
        driveHandler.setDesiredState(DriveHandler.DriveState.AUTOALLIGN);
        climbHandler.setDesiredState(ClimbHandler.ClimbState.OFF);
        break;
      case SPINUPSLOW:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.SLOW);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.AUTOALLIGN);
        climbHandler.setDesiredState(ClimbHandler.ClimbState.OFF);
        break;
      case SLOWSHOT:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.SLOW);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.SLOW);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.SLOWINTAKE);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.SLOWINTAKE);
        driveHandler.setDesiredState(DriveHandler.DriveState.AUTOALLIGN);
        climbHandler.setDesiredState(ClimbHandler.ClimbState.OFF);
        break;
      case SPINUPFAST:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.FAST);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerHighHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        climbHandler.setDesiredState(ClimbHandler.ClimbState.OFF);
         break;
      case FASTSHOT:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.SHOOTING);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.SLOWINTAKE);
        hopperHandler.setDesiredState(HopperHandler.HopperState.FAST);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.FAST);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.FAST);
        driveHandler.setDesiredState(DriveHandler.DriveState.AUTOALLIGN);
        climbHandler.setDesiredState(ClimbHandler.ClimbState.OFF);
        break;
      case CLIMBPREP:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.TELEOPDRIVE);
        climbHandler.setDesiredState(ClimbHandler.ClimbState.LOW);
        break;
      case CLIMBED:
        climbHandler.setDesiredState(ClimbHandler.ClimbState.LOWPULL);
        break;
      case TUNING:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.TUNING);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.FAST);
        indexerHighHandler.setDesiredState(IndexerLowHandler.IndexerLowState.FAST);
        driveHandler.setDesiredState(DriveHandler.DriveState.AUTOALLIGN);
        climbHandler.setDesiredState(ClimbHandler.ClimbState.OFF);
        break;
      case REVERSE:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.FASTREVERSE);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        climbHandler.setDesiredState(ClimbHandler.ClimbState.OFF);
        break;
      case OFF:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.TELEOPDRIVE);
        climbHandler.setDesiredState(ClimbHandler.ClimbState.OFF);
        break;
      default:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.TELEOPDRIVE);
        climbHandler.setDesiredState(ClimbHandler.ClimbState.OFF);
        break;
    }
    currentState = desiredState;
  }

  @Override
  public void periodic() {

        if (shooter == null) {
        System.err.println("ERROR: Superstructure not initialized!");
        return;
        }
        
        if (currentState == SuperstructureState.SPINUP && (shooter.isAtTargetVelo() || drivetrain.isAimedAtTarget())){ //checks if its at target velo and angle
          setDesiredState(SuperstructureState.STATIONARYSHOT);
        }

        if (currentState == SuperstructureState.CLIMBPREP && climbHandler.extendedClimb){
          setDesiredState(SuperstructureState.CLIMBED);
        }

        /*if (currentState == SuperstructureState.CLIMBPREP && climber.atTarget()){
          setDesiredState(SuperS);
        }*/

        SmartDashboard.putBoolean("shooterAtVelo?", shooter.isAtTargetVelo());
        SmartDashboard.putBoolean("Drivetrain aimed?",drivetrain.isAimedAtTarget());
         
  }

  public SuperstructureState getCurrentState() {
    return currentState;
  }

  public SuperstructureState getDesiredState() {
    return desiredState;
  }
}