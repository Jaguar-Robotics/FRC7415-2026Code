// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.handlers.DriveHandler;
import frc.robot.handlers.ShooterHandler;
import frc.robot.handlers.Superstructure;
import frc.robot.handlers.Superstructure.SuperstructureState;
import frc.robot.subsystems.BangBangShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerHighSubsystem;
import frc.robot.subsystems.IndexerLowSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.utils.HubShiftUtil;
import frc.robot.utils.RumbleUtils;

public class RobotContainer {

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.DriveConstants.TranslationDeadband).withRotationalDeadband(MaxAngularRate * Constants.DriveConstants.RotationDeadband) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController opJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public final Vision vision = new Vision(drivetrain);

    public final BangBangShooterSubsystem shooter = new BangBangShooterSubsystem(); 
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final HopperSubsystem hopper = new HopperSubsystem();
    public final IndexerHighSubsystem HighIndexer = new IndexerHighSubsystem();
    public final IndexerLowSubsystem LowIndexer = new IndexerLowSubsystem();
    public final Elevator IntakeSlide = new Elevator();

    public final Superstructure superstructure = Superstructure.getInstance(); 

    Trigger fiveSecWarning = new Trigger(() -> {
    var info = HubShiftUtil.getOfficialShiftInfo();
    return info.remainingTime() <= 5.0;});

    Trigger threeSecWarning = new Trigger(() -> {
        var info = HubShiftUtil.getOfficialShiftInfo();
        return info.remainingTime() <= 3.0;});

    Trigger twoSecWarning = new Trigger(() -> {
        var info = HubShiftUtil.getOfficialShiftInfo();
        return info.remainingTime() <= 2.0;});

    Trigger oneSecWarning = new Trigger(() -> {
        var info = HubShiftUtil.getOfficialShiftInfo();
        return info.remainingTime() <= 1.0;});

    Trigger noButtonsHeld = new Trigger(() ->
    !joystick.a().getAsBoolean() &&
    !joystick.b().getAsBoolean() &&
    !joystick.y().getAsBoolean() &&
    !joystick.x().getAsBoolean() &&
    !joystick.rightTrigger().getAsBoolean() &&
    !joystick.leftTrigger().getAsBoolean() &&
    !joystick.rightBumper().getAsBoolean() &&
    !joystick.leftTrigger().getAsBoolean() &&
    !joystick.rightBumper().getAsBoolean()
    );


    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        DriveHandler.getInstance().initialize(drivetrain, joystick, drive, MaxSpeed, MaxAngularRate);
        ShooterHandler.getInstance().initialize(drivetrain, shooter);
        Superstructure.getInstance().initialize(shooter, drivetrain);

        configurePathPlanner();

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configurePathPlanner() {
        NamedCommands.registerCommand("Intake",
         new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.INTAKE)));
        
        NamedCommands.registerCommand("IntakeOff",
         new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.IDLE)));

        NamedCommands.registerCommand("ShootAuto", 
        new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SPINUPAUTO)));

        NamedCommands.registerCommand("Shoot", 
        new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SPINUP)));

        NamedCommands.registerCommand("ShooterOff", 
        new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.IDLE)));
        
        NamedCommands.registerCommand("ShootSafe", 
        new SequentialCommandGroup(
            new InstantCommand( () -> superstructure.setDesiredState(Superstructure.SuperstructureState.SPINUP)),
            new WaitCommand(2.0),
            new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.STATIONARYSHOT))
        ));

        NamedCommands.registerCommand("ShootTest", 
        new SequentialCommandGroup(
            new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SPINUP)).withTimeout(0.1),
            new WaitCommand(3.0),
            new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.IDLE))
        ));
    }

     private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
       
        /*
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward) DriveStraight = robot centric 
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        ); */
        

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on right stick press.
        joystick.rightStick().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        drivetrain.registerTelemetry(logger::telemeterize);




        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        //ROTATE 90 degreese
        /*
        joystick.leftBumper().onTrue(Commands.runOnce(() -> {
            Rotation2d targetRotation = drivetrain.getPose().getRotation().plus(Rotation2d.fromDegrees(90));
            
            // Use your existing rotation controller
            drivetrain.applyRequest(() -> {
                double rotationalRate = CommandSwerveDrivetrain.rotationController.calculate(
                    drivetrain.getPose().getRotation().getRadians(),
                    targetRotation.getRadians()
                );
                return new SwerveRequest.FieldCentric()
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(rotationalRate * 6); // Max angular rate
            }).withTimeout(2.0).schedule();
        }));
        */

        /* Main driver Controller:
         * RT - Hold to spin up (and shoot hopefully) - relase to idle
         * RB - Shoot (dont use unless robot broken)
         * LT - Hold to Intake - release to idle
         * B - Spin up Fast (use if broken)
         * Y - FAST SHOT (use if broken)
         * A - Hold to bump assist - release to idle
         * X - Robot off
         * Left Stick in - Reverse shi 
         */
        joystick.rightTrigger().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SPINUP)));           

        //joystick.rightBumper().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.STATIONARYSHOT)));
        joystick.rightBumper().onTrue(new SequentialCommandGroup(
            Commands.runOnce(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SPINUPFAST)),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.FASTSHOT))));

        //joystick.leftBumper().onTrue(new InstantCommand(() -:drivetrain.setDefaultCommand(drivetrain.headingLocktoHub(joystick, MaxSpeed, MaxAngularRate, "no"))); //shoot while stationary
        
        joystick.b().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SPINUPFAST)));
        joystick.y().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.FASTSHOT)));

        joystick.a().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState. BUMP)));

        joystick.x().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.OFF)));

        joystick.leftTrigger().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.INTAKESLOW)));
        joystick.leftBumper().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.INTAKESLOWSLOW)));

        joystick.leftStick().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.REVERSE)));

        noButtonsHeld.onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.IDLE)));
        

       //joystick.a().onTrue(new InstantCommand(() -> superstructure.setDesiredState((Superstructure.SuperstructureState.TUNING))));
       //joystick.a().onFalse(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.OFF)));


        

        joystick.rightStick().onTrue(new InstantCommand(() -> drivetrain.seedFieldCentric()));


        
        //joystick.povDown().onTrue(Commands.runOnce(() -> IntakeSlideHandler.getInstance().setDesiredState(IntakeSlideState.REZEROIN))); 
        //joystick.povUp().onTrue(Commands.runOnce(() -> IntakeSlideHandler.getInstance().setDesiredState(IntakeSlideState.REZEROOUT))); //in RPM
        

        joystick.povRight().whileTrue(IntakeSlide.manualDrive(() -> 0.67)); //  out
        joystick.povLeft().whileTrue(IntakeSlide.manualDrive(() -> -0.67)); //in


        //CONTROLLER 2 / debug controller 
        /*
         *  RT - Shoot
         *  B - re-zero intake IN
         *  Y - overide alliance winner (on a switch)
         *  DPAD - shift hub by 0.1 M in direction (up is away, down is closer)
         *  A - Reset Shifted Hub to where it should be
         */

        opJoystick.rightTrigger().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.STATIONARYSHOT)));
        opJoystick.b().onTrue(new InstantCommand(() -> IntakeSlide.calibrateZeroIn()));
        //AUTO WINNER OVERIDES
        opJoystick.y().onTrue(Commands.runOnce(() -> {
            var current = HubShiftUtil.getAllianceWinOverride();
            HubShiftUtil.setAllianceWinOverride(() -> Optional.of(current.orElse(true) == false));
        }));

        opJoystick.a().onTrue(Commands.runOnce(() -> drivetrain.resetHubOffset()));
        opJoystick.a().onTrue(Commands.runOnce(() -> shooter.resetShooterMult()));

        // Change Shooter Power
        opJoystick.povUp().onTrue(Commands.runOnce(() -> shooter.changeShooterMult(0.05)));
        opJoystick.povDown().onTrue(Commands.runOnce(() -> shooter.changeShooterMult(-0.05)));

        // Adjust Y offset
        opJoystick.povRight().onTrue(Commands.runOnce(() -> drivetrain.setHubOffset(0.0, 0.1)));
        opJoystick.povLeft().onTrue(Commands.runOnce(() -> drivetrain.setHubOffset(0.0, -0.1)));

        opJoystick.x().onTrue(Commands.runOnce(() -> shooter.toggleShooterMult()));

        /* 
        opJoystick.a().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.TUNING)));
        joystick.povUp().onTrue(Commands.runOnce(() -> ShooterHandler.getInstance().adjustFastShot(1)));
        joystick.povDown().onTrue(Commands.runOnce(() -> ShooterHandler.getInstance().adjustFastShot(-1)));
        */


        RobotModeTriggers.teleop().onTrue(Commands.runOnce(HubShiftUtil::initialize));
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(HubShiftUtil::initialize));

        fiveSecWarning.onTrue(RumbleUtils.rumble(joystick, 0.5, 0.5));
        threeSecWarning.onTrue(RumbleUtils.rumble(joystick, 0.5, 0.25));
        twoSecWarning.onTrue(RumbleUtils.rumble(joystick, 0.5, 0.25));
        oneSecWarning.onTrue(RumbleUtils.rumble(joystick, 0.5, 1));

        fiveSecWarning.onTrue(new InstantCommand(() -> SmartDashboard.putBoolean("isTsWorking", true)));
    }        

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
