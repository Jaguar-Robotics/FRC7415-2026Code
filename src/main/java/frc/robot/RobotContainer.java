// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import org.w3c.dom.traversal.TreeWalker;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.handlers.DriveHandler;
import frc.robot.handlers.IntakeHandler;
import frc.robot.handlers.ShooterHandler;
import frc.robot.handlers.ClimbHandler;
import frc.robot.handlers.Superstructure;
import frc.robot.handlers.Superstructure.SuperstructureState;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerHighSubsystem;
import frc.robot.subsystems.IndexerLowSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;
import pabeles.concurrency.IntOperatorTask.Max;

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

    public final ShooterSubsystem shooter = new ShooterSubsystem(); 
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final HopperSubsystem hopper = new HopperSubsystem();
    public final IndexerHighSubsystem HighIndexer = new IndexerHighSubsystem();
    public final IndexerLowSubsystem LowIndexer = new IndexerLowSubsystem();
    public final ClimbSubsystem climb = new ClimbSubsystem();


    public final Superstructure superstructure = Superstructure.getInstance(); 

    //private TalonFX intakeMotor = new TalonFX(Constants.IntakeConstants.IntakeMotorID);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        DriveHandler.getInstance().initialize(drivetrain, joystick, drive, MaxSpeed, MaxAngularRate);
        ShooterHandler.getInstance().initialize(drivetrain, shooter);
        Superstructure.getInstance().initialize(shooter, drivetrain, climb);
        ShooterSubsystem.getInstance().initialize(drivetrain);

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
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

        PathConstraints constraints = new PathConstraints(3, 4,
        Degrees.of(540).in(Radians), Degrees.of(720).in(Radians));

        joystick.start().whileTrue(drivetrain.getSnakeDriveCommand(drive, drivetrain, joystick, MaxSpeed, MaxAngularRate));

        
        //joystick.rightBumper().whileTrue(drivetrain.shootOnTheMoveIterative(joystick, MaxSpeed, MaxAngularRate, "no")); //Shoot while moving
        joystick.rightBumper().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SPINUP)));
        joystick.rightBumper().onFalse(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.IDLE)));

        //joystick.leftBumper().onTrue(new InstantCommand(() -:drivetrain.setDefaultCommand(drivetrain.headingLocktoHub(joystick, MaxSpeed, MaxAngularRate, "no"))); //shoot while stationary
        
        joystick.b().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SPINUP)));
        
        joystick.x().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.OFF)));

        joystick.leftTrigger().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.INTAKE)));
        joystick.leftTrigger().onFalse(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.OFF)));

        joystick.leftStick().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.REVERSE)));

        joystick.a().onTrue(new InstantCommand(() -> superstructure.setDesiredState((Superstructure.SuperstructureState.TUNING))));

        joystick.rightStick().onTrue(new InstantCommand(() -> drivetrain.seedFieldCentric()));

        joystick.leftBumper().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.CLIMBPREP)));
        //FOR HESHEL
        /*
        joystick.leftTrigger().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.INTAKE)));
        joystick.leftTrigger().onFalse(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.OFF)));
        
        joystick.a().onTrue(
            new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SPINUPSLOW))
            .andThen(new WaitCommand(1.0))
            .andThen(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SLOWSHOT))));
        joystick.y().onTrue(
            new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SPINUPFAST))
            .andThen(new WaitCommand(1.0))
            .andThen(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.FASTSHOT))));
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

        
        
        

        
        joystick.povDown().onTrue(Commands.runOnce(() -> ShooterHandler.getInstance().adjustFastShot(-250))); //in RPM
        joystick.povUp().onTrue(Commands.runOnce(() -> ShooterHandler.getInstance().adjustFastShot(250)));
        
        joystick.povLeft().whileTrue(drivetrain.shootOnTheMoveIterative(joystick, MaxSpeed, MaxAngularRate, "PovLeft")); //Shoot while moving
        
        joystick.povRight().whileTrue(drivetrain.shootOnTheMoveIterative(joystick, MaxSpeed, MaxAngularRate, "PovRight")); //Shoot while moving
    
    }        

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
