package frc.robot.handlers;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.BangBangShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Superstructure extends SubsystemBase {

  // Define your states
  public enum SuperstructureState {
    STATIONARYSHOT,
    SHOOTONTHEMOVE,
    SHOOTONTHEMOVESPINUP,
    INTAKE,
    INTAKESLOW,
    INTAKEFAST,
    INTAKESNAKE,
    INTAKESNAKEFAST,
    SLOWSHOT,
    REVERSE,
    SPINUP,
    TUNING,
    OFF,
    IDLE,
    SPINUPAUTO,
    STATIONARYSHOTAUTO,
    SOTMSPINUPAUTO,
    SOTMAUTO,
    SPINUPSLOW,
    SPINUPFAST,
    FASTSHOT,
    AIM,
    BUMP
  }

  private static Superstructure instance;
  private BangBangShooterSubsystem shooter; 
  private CommandSwerveDrivetrain drivetrain;
  private CommandXboxController joystick;
  
  //private final ShooterSubsystem Shooter = new ShooterSubsystem();
  private final ShooterHandler shooterHandler = ShooterHandler.getInstance();

  //private final IntakeSubsystem Intake = new IntakeSubsystem();
  private final IntakeHandler intakeHandler = IntakeHandler.getInstance();

  private final IntakeSlideHandler intakeSlideHandler = IntakeSlideHandler.getInstance();

  //private final HopperSubsystem Hopper = new HopperSubsystem();
  private final HopperHandler hopperHandler = HopperHandler.getInstance();

  //private final IndexerSubsystem Indexer = new IndexerSubsystem();
  private final IndexerHighHandler indexerHighHandler = IndexerHighHandler.getInstance();
  private final IndexerLowHandler indexerLowHandler = IndexerLowHandler.getInstance();

  private final DriveHandler driveHandler = DriveHandler.getInstance();

  private SuperstructureState desiredState = SuperstructureState.IDLE;
  private SuperstructureState currentState = SuperstructureState.IDLE;
  private Angle targetAngle = Degrees.of(0);
  private final SendableChooser<Boolean> IsTallChooser = new SendableChooser<>();

  boolean BumpHeight = true;
  boolean DTaimed;
  boolean DTAUTOaimed;
  boolean DTFutAimed;
  double robotSpeed;
  boolean ShooterAtVelo;
  boolean driverIsMoving;
  boolean isMoving;
  ChassisSpeeds currentSpeeds;

  private final edu.wpi.first.wpilibj.Timer spinupTimer = new edu.wpi.first.wpilibj.Timer();
  

  public static Superstructure getInstance(){
    if (instance == null) {
      instance = new Superstructure();
    }
    return instance;
  }

    /**
   * Initialize superstructure with required subsystems
   */
  public void initialize(BangBangShooterSubsystem shooter, CommandSwerveDrivetrain drivetrain, CommandXboxController joystick) {
      this.shooter = shooter;
      this.drivetrain = drivetrain;
      this.joystick = joystick;

      // Set up the chooser options
    IsTallChooser.setDefaultOption("Bump Height", true);
    IsTallChooser.addOption("Trench Height", false);
    
    // Put the chooser on the dashboard
    SmartDashboard.putData("height Chooser", IsTallChooser);

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
        intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.BRAKE);
        break;
      case INTAKE:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.FASTINTAKE);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.TELEOPDRIVE);
        intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.OUT);
        break;
      case INTAKESLOW:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.FASTINTAKE);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.TELEOPDRIVESLOW);
        intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.OUT);
        break;
        case INTAKEFAST:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.MAXSPEED);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.OUT);
        break;
      case INTAKESNAKE:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.FASTINTAKE);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.SNAKE);
        intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.OUT);
        break;
      case INTAKESNAKEFAST:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.MAXSPEED);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.SNAKE);
        intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.OUT);
        break;
      case SPINUP:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.SHOOTING); 
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.AUTOALLIGN);
        ShooterAtVelo = false;
        spinupTimer.reset();
        spinupTimer.start();
        break;
      case SPINUPAUTO: //spinup w/o drivehandler
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.SHOOTING); 
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        ShooterAtVelo = false;
        spinupTimer.reset();
        spinupTimer.start();
        break;
      case STATIONARYSHOT:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.SHOOTING);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.SLOWINTAKE);
        hopperHandler.setDesiredState(HopperHandler.HopperState.FAST);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.FAST);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.FAST);
        driveHandler.setDesiredState(DriveHandler.DriveState.XDRIVE);
          if (BumpHeight) {intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.SLOWIN);}
          else intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.FASTSLOWIN);
        break;
      case STATIONARYSHOTAUTO:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.SHOOTING);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.FAST);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.SLOWINTAKE);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.SLOWINTAKE);
        if (BumpHeight) {intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.SLOWIN);}
        else intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.FASTSLOWIN);
        break;
      case SHOOTONTHEMOVE:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.SOTM);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.SLOWINTAKE);
        hopperHandler.setDesiredState(HopperHandler.HopperState.FAST);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.FAST);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.FAST);
        driveHandler.setDesiredState(DriveHandler.DriveState.SHOOTONTHEMOVE);
        intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.OUT);
        break;
      case SHOOTONTHEMOVESPINUP:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.SOTM);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.SLOWINTAKE);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.SHOOTONTHEMOVE);
        spinupTimer.reset();
        spinupTimer.start();
        break;
      case SOTMSPINUPAUTO:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.SOTM);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        spinupTimer.reset();
        spinupTimer.start();
        break;
      case SOTMAUTO:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.SOTM);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.FAST);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.SLOWINTAKE);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.SLOWINTAKE);
        intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.SLOWIN);
        break;
      case TUNING: //dont use
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.TUNING);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.FAST);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.FAST);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.FAST);
        driveHandler.setDesiredState(DriveHandler.DriveState.AUTOALLIGN);
        break;
      case SLOWSHOT: //dont use
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.SLOW);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.SLOW);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.SLOWINTAKE);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.SLOWINTAKE);
        //driveHandler.setDesiredState(DriveHandler.DriveState.AUTOALLIGN);
        break;
      case SPINUPFAST: //dont use
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.FAST); //change to shooting
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.AUTOALLIGN);
        ShooterAtVelo = false;
         break;
      case FASTSHOT: //dont use
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.FAST);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.FAST);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.SLOWINTAKE);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.SLOWINTAKE);
        driveHandler.setDesiredState(DriveHandler.DriveState.AUTOALLIGN);
        intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.SLOWIN);
        break;
      case REVERSE:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.FASTREVERSE);
        hopperHandler.setDesiredState(HopperHandler.HopperState.FASTOUT);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.SLOWREVERSE);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.FASTREVERSE);
        intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.OUT);
        break;
      case AIM:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.AUTOALLIGN);
        break;
      case BUMP:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.BUMP_LOCK);
        break;
      case OFF:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.TELEOPDRIVE);
        intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.IN);
        break;
      default:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.TELEOPDRIVE);
        intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.IN);
        break;
    }
    currentState = desiredState;
  }

  @Override
  public void periodic() {

    BumpHeight = IsTallChooser.getSelected();

    double joystickMagnitude = Math.hypot(
    joystick.getLeftX(), 
    joystick.getLeftY()
);
  

    SmartDashboard.putBoolean("BumpHeight?", BumpHeight);

        if (shooter == null) {
        System.err.println("ERROR: Superstructure not initialized!");
        return;
        }

        currentSpeeds = drivetrain.getState().Speeds;
        robotSpeed = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        driverIsMoving = joystickMagnitude > Constants.DriveConstants.TranslationDeadband; // matches your deadband
        DTaimed = drivetrain.isAimedAtTarget();
        DTAUTOaimed = drivetrain.isAimedAtTargetAuto();
        DTFutAimed = drivetrain.isAimedAtTargetSOTM();

          if (CommandSwerveDrivetrain.isInAllianceZone(drivetrain.getPose())){
              ShooterAtVelo = shooter.atTargetVelo();}
          else {
              ShooterAtVelo = shooter.atTargetVeloPassing();}
        
        if (currentState == SuperstructureState.SPINUP && ShooterAtVelo && DTaimed && spinupTimer.hasElapsed(0.05)){ //checks if its at target velo and angle
          setDesiredState(SuperstructureState.STATIONARYSHOT);
        }
        /* I do ts by waiting for 0.5 sec spinup
        if (currentState == SuperstructureState.SPINUPAUTO && ShooterAtVelo && DTAUTOaimed && spinupTimer.hasElapsed(0.05)){ //checks if its at target velo and angle
          setDesiredState(SuperstructureState.STATIONARYSHOTAUTO);
        } */

        if (currentState == SuperstructureState.SHOOTONTHEMOVESPINUP && ShooterAtVelo && DTFutAimed && spinupTimer.hasElapsed(0.05)){
          setDesiredState(SuperstructureState.SHOOTONTHEMOVE);
        }

        if (currentState == SuperstructureState.SOTMSPINUPAUTO && ShooterAtVelo && DTFutAimed && spinupTimer.hasElapsed(0.05)){
          setDesiredState(SuperstructureState.SOTMAUTO);
        }

        if (currentState == SuperstructureState.SHOOTONTHEMOVE && (!DTFutAimed || !ShooterAtVelo)){
          setDesiredState(SuperstructureState.SHOOTONTHEMOVESPINUP);
        }

        // STATIONARYSHOT → SHOOTONTHEMOVE if robot starts moving
        if ((currentState == SuperstructureState.STATIONARYSHOT || currentState == SuperstructureState.SPINUP) && driverIsMoving) {
            setDesiredState(SuperstructureState.SHOOTONTHEMOVESPINUP);
        }

        // SHOOTONTHEMOVE → SPINUP (which auto-transitions to STATIONARYSHOT) if robot stops
        if ((currentState == SuperstructureState.SHOOTONTHEMOVE || currentState == SuperstructureState.SHOOTONTHEMOVESPINUP) && !driverIsMoving) {
          setDesiredState(SuperstructureState.SPINUP);
        }

        /*
        if (currentState == SuperstructureState.STATIONARYSHOT && !DTaimed){
          setDesiredState(SuperstructureState.SPINUP);
        } */
          


        SmartDashboard.putString("SuperState", currentState.toString());
        SmartDashboard.putBoolean("shooterAtVelo?", ShooterAtVelo);
        SmartDashboard.putBoolean("Drivetrain aimed?",DTaimed);
        SmartDashboard.putBoolean("SOTM aimed?",DTFutAimed);
         
  }
  public SuperstructureState getCurrentState() {
    return currentState;
  }

  public SuperstructureState getDesiredState() {
    return desiredState;
  }
}