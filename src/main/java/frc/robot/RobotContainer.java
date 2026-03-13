// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.ejml.sparse.csc.linsol.qr.LinearSolverQrLeftLooking_DSCC;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.OneShotTriggerEvent;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.handlers.DriveHandler;
import frc.robot.handlers.IntakeHandler;
import frc.robot.handlers.ShooterHandler;
import frc.robot.handlers.Superstructure;
import frc.robot.handlers.IntakeHandler.IntakeState;
import frc.robot.handlers.IntakeSlideHandler;
import frc.robot.handlers.IntakeSlideHandler.IntakeSlideState;
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

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public final Vision vision = new Vision(drivetrain);

    public final BangBangShooterSubsystem shooter = new BangBangShooterSubsystem(); 
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final HopperSubsystem hopper = new HopperSubsystem();
    public final IndexerHighSubsystem HighIndexer = new IndexerHighSubsystem();
    public final IndexerLowSubsystem LowIndexer = new IndexerLowSubsystem();
    public final Elevator IntakeSlide = new Elevator();

    public final Superstructure superstructure = Superstructure.getInstance(); 

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

        NamedCommands.registerCommand("Shoot", 
        new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SPINUP)));
        
        NamedCommands.registerCommand("ShootSafe", 
        new SequentialCommandGroup(
            new InstantCommand( () -> superstructure.setDesiredState(Superstructure.SuperstructureState.SPINUP)),
            Commands.waitSeconds(2),
            new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.STATIONARYSHOT))
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

        
       
        joystick.rightTrigger().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SPINUP)));           
        
        joystick.rightTrigger().onFalse(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.IDLE)));

        //joystick.leftBumper().onTrue(new InstantCommand(() -:drivetrain.setDefaultCommand(drivetrain.headingLocktoHub(joystick, MaxSpeed, MaxAngularRate, "no"))); //shoot while stationary
        
        joystick.b().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SPINUPFAST)));
        joystick.y().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.FASTSHOT)));

        joystick.a().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.TUNING)));
        
        joystick.x().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.OFF)));

        joystick.leftTrigger().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.INTAKE)));
        joystick.leftTrigger().onFalse(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.IDLE)));

        joystick.leftStick().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.REVERSE)));
        

       //joystick.a().onTrue(new InstantCommand(() -> superstructure.setDesiredState((Superstructure.SuperstructureState.TUNING))));
       //joystick.a().onFalse(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.OFF)));


        

        joystick.rightStick().onTrue(new InstantCommand(() -> drivetrain.seedFieldCentric()));

        

        
        //FOR HESHEL
        /*
        joystick.leftTrigger().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.INTAKE)));
        joystick.leftTrigger().onFalse(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.OFF)));
        
        joystick.a().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SPINUPFAST)));
        joystick.y().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.FASTSHOT)));
        joystick.x().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.OFF)));  
        */


        /*BINDS: 
        Right bumper = auto angle
        B = spinup
        Y = Shoot with auto shooter speed 
        Left Trigger = hold for intake/ release for idle
        Left stick in = reverse (unstuck shi)
        A = Tuning mode (Dpad up or down to change speed by 250 RPM)
        */

        
        
        

        
        //joystick.povDown().onTrue(Commands.runOnce(() -> IntakeSlideHandler.getInstance().setDesiredState(IntakeSlideState.REZEROIN))); 
        //joystick.povUp().onTrue(Commands.runOnce(() -> IntakeSlideHandler.getInstance().setDesiredState(IntakeSlideState.REZEROOUT))); //in RPM
        joystick.povUp().onTrue(Commands.runOnce(() -> ShooterHandler.getInstance().adjustFastShot(1)));
        joystick.povDown().onTrue(Commands.runOnce(() -> ShooterHandler.getInstance().adjustFastShot(-1)));
        
        //joystick.povLeft().onTrue(IntakeSlide.goToSetpoint(()-> Elevator.Setpoint.OUT));
        //joystick.povRight().onTrue(IntakeSlide.goToSetpoint(()-> Elevator.Setpoint.IN));

        joystick.povRight().whileTrue(IntakeSlide.manualDrive(() -> 0.67)); //  out
        joystick.povLeft().whileTrue(IntakeSlide.manualDrive(() -> -0.67)); //in

        Runnable rumbleOn  = () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
        Runnable rumbleOff = () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);

        new Trigger(() -> {
            var info = HubShiftUtil.getOfficialShiftInfo();
            if (info == null) return false;
            double r = info.remainingTime();
            return r <= 5.0 && r > 4.5;
        }).whileTrue(Commands.run(rumbleOn)).onFalse(Commands.runOnce(rumbleOff));

        new Trigger(() -> {
            var info = HubShiftUtil.getOfficialShiftInfo();
            if (info == null) return false;
            double r = info.remainingTime();
            return r <= 3.0 && r > 2.5;
        }).whileTrue(Commands.run(rumbleOn)).onFalse(Commands.runOnce(rumbleOff));

        new Trigger(() -> {
            var info = HubShiftUtil.getOfficialShiftInfo();
            if (info == null) return false;
            double r = info.remainingTime();
            return r <= 1.0 && r > 0.0;
        }).whileTrue(Commands.run(rumbleOn)).onFalse(Commands.runOnce(rumbleOff));

        new Trigger(() -> {
            double remaining = HubShiftUtil.getOfficialShiftInfo().remainingTime();
            return remaining <= 1.0 && HubShiftUtil.isNextShiftActive();
        }).onTrue(Commands.runOnce(() -> {
            System.out.println("Early shoot window opened, remaining: " + HubShiftUtil.getOfficialShiftInfo().remainingTime());
        }));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}