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
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
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
import frc.robot.generated.TunerConstants;
import frc.robot.handlers.DriveHandler;
import frc.robot.handlers.IntakeSlideHandler;
import frc.robot.handlers.IntakeSlideHandler.IntakeSlideState;
import frc.robot.handlers.ShooterHandler;
import frc.robot.handlers.Superstructure;
import frc.robot.subsystems.BangBangShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerHighSubsystem;
import frc.robot.subsystems.IndexerLowSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.utils.HubShiftUtil;
import frc.robot.utils.RumbleUtils;

public class RobotContainer {

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.DriveConstants.TranslationDeadband).withRotationalDeadband(Constants.DriveConstants.RotationDeadband) // Add a 10% deadband
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
    !joystick.rightBumper().getAsBoolean() &&
    !joystick.leftTrigger().getAsBoolean() &&
    !joystick.leftBumper().getAsBoolean()
    );


    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        PPHolonomicDriveController.clearRotationFeedbackOverride();
        DriveHandler.getInstance().initialize(drivetrain, joystick, drive, MaxSpeed, MaxAngularRate);
        ShooterHandler.getInstance().initialize(drivetrain, shooter);
        Superstructure.getInstance().initialize(shooter, drivetrain, joystick);
        BangBangShooterSubsystem.getInstance().initialize(drivetrain);

        configurePathPlanner();

        autoChooser = AutoBuilder.buildAutoChooser("Tests");

        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configurePathPlanner() {
        NamedCommands.registerCommand("Intake",
         new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.INTAKEFAST)));
        
        NamedCommands.registerCommand("IntakeOff",
         new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.IDLE)));

        NamedCommands.registerCommand("AutoShoot",
        new SequentialCommandGroup(
            new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SPINUPAUTO)).withTimeout(0.1),
            new WaitCommand(0.3),
            new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.STATIONARYSHOTAUTO))
        ));
        
        NamedCommands.registerCommand("StartHubAlign", Commands.runOnce(() -> {
            PPHolonomicDriveController.overrideRotationFeedback(() -> {
                Pose2d currentPose = drivetrain.getPose();
                Pose2d targetPose = drivetrain.getHubPose().toPose2d();
                if (targetPose == null) return 0.0;

                Translation2d toTarget = currentPose.getTranslation().minus(targetPose.getTranslation());
                Rotation2d desiredAngle = toTarget.getAngle().rotateBy(Rotation2d.k180deg);

                // rotationController already has enableContinuousInput — use it directly
                double pidOutput = CommandSwerveDrivetrain.rotationController.calculate(
                    currentPose.getRotation().getRadians(),
                    desiredAngle.getRadians()
                );

                SmartDashboard.putNumber("AutoStuff/headingCalculated", pidOutput);
                SmartDashboard.putNumber("AutoStuff/currentPoseGetRotation", currentPose.getRotation().getDegrees());
                SmartDashboard.putNumber("AutoStuff/desiredAngleRotation", desiredAngle.getDegrees());

                return pidOutput * MaxAngularRate * 5;
            });
        }));
    
        /*
        NamedCommands.registerCommand("StartSOTMAlign", Commands.runOnce(() -> {
            drivetrain.lastAimAngleRad = Double.NaN;
            drivetrain.driveAngleFilter.reset();
            drivetrain.lastCommandedSpeeds = new ChassisSpeeds();

            drivetrain.vxFilter.reset();
            drivetrain.vyFilter.reset();
            drivetrain.omegaFilter.reset();

        superstructure.setDesiredState(Superstructure.SuperstructureState.SOTMSPINUPAUTO);
        PPHolonomicDriveController.overrideRotationFeedback(() -> {

            // ── 1. CURRENT STATE ──────────────────────────────────────────────
            Pose2d currentPose = drivetrain.getPose();

            ChassisSpeeds measuredSpeeds = drivetrain.getState().Speeds;
            ChassisSpeeds robotRelativeSpeeds = 
                (drivetrain.lastCommandedSpeeds.vxMetersPerSecond == 0
                && drivetrain.lastCommandedSpeeds.vyMetersPerSecond == 0
                && drivetrain.lastCommandedSpeeds.omegaRadiansPerSecond == 0)
                ? measuredSpeeds
                : drivetrain.lastCommandedSpeeds;

                
            ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeSpeeds.vxMetersPerSecond,
                robotRelativeSpeeds.vyMetersPerSecond,
                robotRelativeSpeeds.omegaRadiansPerSecond,
                currentPose.getRotation()
            );
            // ── 2. PHASE DELAY ────────────────────────────────────────────────
            double phaseDelay = 0.03;
            Pose2d estimatedPose = currentPose.exp(
                new Twist2d(
                    fieldVelocity.vxMetersPerSecond * phaseDelay,
                    fieldVelocity.vyMetersPerSecond * phaseDelay,
                    fieldVelocity.omegaRadiansPerSecond * phaseDelay
                )
            );

            // ── 3. SHOOTER POSITION ───────────────────────────────────────────
            Translation2d shooterOffsetFieldFrame = 
                CommandSwerveDrivetrain.ROBOT_TO_SHOOTER.rotateBy(estimatedPose.getRotation());
            Translation2d shooterPosition = 
                estimatedPose.getTranslation().plus(shooterOffsetFieldFrame);

            // ── 4. TARGET ─────────────────────────────────────────────────────
            Translation2d target = drivetrain.getHubPose().toPose2d().getTranslation();

            // ── 5. SHOOTER VELOCITY ───────────────────────────────────────────

            double omega = fieldVelocity.omegaRadiansPerSecond;
            // Tangential velocity added by rotation: perpendicular to the offset vector
            // If offset is (rx, ry), tangential is (-ry*omega, rx*omega)
            double shooterVelocityX = fieldVelocity.vxMetersPerSecond
                    + (-shooterOffsetFieldFrame.getY() * omega);
            double shooterVelocityY = fieldVelocity.vyMetersPerSecond
                    + (shooterOffsetFieldFrame.getX() * omega);

            boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
            double allianceFlip = isRed ? -1.0 : 1.0;

            shooterVelocityX = fieldVelocity.vxMetersPerSecond;// * allianceFlip; //if allanice flup put it here
            shooterVelocityY = fieldVelocity.vyMetersPerSecond;// * allianceFlip;

            // ── 6. ITERATIVE LOOKAHEAD LOOP ───────────────────────────────────
            double timeOfFlight = 0.1;
            Translation2d lookaheadShooterPosition = shooterPosition;
            double lookaheadDistance = target.getDistance(shooterPosition);

            for (int i = 0; i < 20; i++) {
                double distInches = lookaheadDistance * 39.3701;
                timeOfFlight = CommandSwerveDrivetrain.TOFmap.get(distInches);
                timeOfFlight = Math.max(timeOfFlight, 0.05);

                if(RobotBase.isSimulation()){ timeOfFlight = 1;}

                double offsetX = shooterVelocityX * timeOfFlight; // *0.15
                double offsetY = shooterVelocityY * timeOfFlight;
                lookaheadShooterPosition = shooterPosition.plus(new Translation2d(offsetX, offsetY));
                lookaheadDistance = target.getDistance(lookaheadShooterPosition);
            }

            // ── 7. AIM ANGLE ──────────────────────────────────────────────────
            Rotation2d aimAngle = target.minus(lookaheadShooterPosition).getAngle();
            double aimAngleRad = aimAngle.getRadians();

            // ── 8. AIM ANGLE FEEDFORWARD ──────────────────────────────────────
            double aimAngleVelocityFF = 0.0;
            if (!Double.isNaN(drivetrain.lastAimAngleRad)) {
                double rawDelta = aimAngleRad - drivetrain.lastAimAngleRad;
                while (rawDelta > Math.PI)  rawDelta -= 2 * Math.PI;
                while (rawDelta < -Math.PI) rawDelta += 2 * Math.PI;

                double rawRate = rawDelta / 0.02;
                aimAngleVelocityFF = drivetrain.driveAngleFilter.calculate(rawRate);
            }
            drivetrain.lastAimAngleRad = aimAngleRad;

            drivetrain.ShootingLocation = new Pose2d(lookaheadShooterPosition, aimAngle);


            // ── 9. ROTATION PID + FF ──────────────────────────────────────────
            double pidOutput = drivetrain.rotationController.calculate(
                currentPose.getRotation().getRadians(),
                aimAngleRad
            );

            double kFF = 0.35; //0.25
            double rotationalRate = aimAngleVelocityFF * kFF + pidOutput;

            // ── 10. SMARTDASHBOARD ────────────────────────────────────────────
            SmartDashboard.putNumber("AutoSOTM/AimAngleDeg", Math.toDegrees(aimAngleRad));
            SmartDashboard.putNumber("AutoSOTM/LookaheadDistanceInches", lookaheadDistance * 39.3701);
            SmartDashboard.putNumber("AutoSOTM/TimeOfFlight", timeOfFlight);
            SmartDashboard.putNumber("AutoSOTM/pidOutput", pidOutput);
            SmartDashboard.putNumber("AutoSOTM/rotationalRate", rotationalRate);

            return rotationalRate * MaxAngularRate;
        });
    }));  */

    NamedCommands.registerCommand("StartSOTMAlign", Commands.runOnce(() -> {
    drivetrain.lastAimAngleRad = Double.NaN;
    drivetrain.driveAngleFilter.reset();
    drivetrain.lastCommandedSpeeds = new ChassisSpeeds();
    
    superstructure.setDesiredState(Superstructure.SuperstructureState.SOTMSPINUPAUTO);
    
    PPHolonomicDriveController.overrideRotationFeedback(() -> {
        Pose2d currentPose = drivetrain.getPose();

        ChassisSpeeds measuredSpeeds = drivetrain.getState().Speeds;
        ChassisSpeeds robotRelativeSpeeds = 
            (drivetrain.lastCommandedSpeeds.vxMetersPerSecond == 0
            && drivetrain.lastCommandedSpeeds.vyMetersPerSecond == 0
            && drivetrain.lastCommandedSpeeds.omegaRadiansPerSecond == 0)
            ? measuredSpeeds
            : drivetrain.lastCommandedSpeeds;

        ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds.vxMetersPerSecond,
            robotRelativeSpeeds.vyMetersPerSecond,
            robotRelativeSpeeds.omegaRadiansPerSecond,
            currentPose.getRotation()
        );

        // REMOVE PHASE DELAY — in auto, just use current pose
        // No estimated pose, no velocity lookahead for position
        Translation2d shooterOffsetFieldFrame = 
            CommandSwerveDrivetrain.ROBOT_TO_SHOOTER.rotateBy(currentPose.getRotation());
        Translation2d shooterPosition = currentPose.getTranslation().plus(shooterOffsetFieldFrame);

        Translation2d target = drivetrain.getTargetPose(currentPose).getTranslation();

        double omega = fieldVelocity.omegaRadiansPerSecond;
        double shooterVelocityX = fieldVelocity.vxMetersPerSecond
                + (-shooterOffsetFieldFrame.getY() * omega);
        double shooterVelocityY = fieldVelocity.vyMetersPerSecond
                + (shooterOffsetFieldFrame.getX() * omega);

        double dampFactor = 0.3;  // tune this: lower = more dampening, higher = more responsive
        shooterVelocityX = shooterVelocityX * dampFactor + (drivetrain.lastShooterVelocityX * (1.0 - dampFactor));
        shooterVelocityY = shooterVelocityY * dampFactor + (drivetrain.lastShooterVelocityY * (1.0 - dampFactor));

        // Cache for next loop
        drivetrain.lastShooterVelocityX = shooterVelocityX;
        drivetrain.lastShooterVelocityY = shooterVelocityY;

        double timeOfFlight = 0.1;
        Translation2d lookaheadShooterPosition = shooterPosition;
        double lookaheadDistance = target.getDistance(shooterPosition);

        for (int i = 0; i < 20; i++) {
            double distInches = lookaheadDistance * 39.3701;
            timeOfFlight = CommandSwerveDrivetrain.TOFmap.get(distInches);
            timeOfFlight = Math.max(timeOfFlight, 0.05);

            double offsetX = shooterVelocityX * timeOfFlight;
            double offsetY = shooterVelocityY * timeOfFlight;
            lookaheadShooterPosition = shooterPosition.plus(new Translation2d(offsetX, offsetY));
            lookaheadDistance = target.getDistance(lookaheadShooterPosition);
        }

        Rotation2d aimAngle = target.minus(lookaheadShooterPosition).getAngle();
        double aimAngleRad = aimAngle.getRadians();

        double aimAngleVelocityFF = 0.0;
        if (!Double.isNaN(drivetrain.lastAimAngleRad)) {
            double rawDelta = aimAngleRad - drivetrain.lastAimAngleRad;
            while (rawDelta > Math.PI)  rawDelta -= 2 * Math.PI;
            while (rawDelta < -Math.PI) rawDelta += 2 * Math.PI;
            double rawRate = rawDelta / 0.02;
            aimAngleVelocityFF = drivetrain.driveAngleFilter.calculate(rawRate);
        }
        drivetrain.lastAimAngleRad = aimAngleRad;
        drivetrain.ShootingLocation = new Pose2d(lookaheadShooterPosition, aimAngle);

        double pidOutput = drivetrain.rotationController.calculate(
            currentPose.getRotation().getRadians(),
            aimAngleRad
        );

        double kFF = 0.25;
        double rotationalRate = aimAngleVelocityFF * kFF + pidOutput;

        SmartDashboard.putNumber("AutoSOTM/AimAngleDeg", Math.toDegrees(aimAngleRad));
        SmartDashboard.putNumber("AutoSOTM/LookaheadDistanceInches", lookaheadDistance * 39.3701);
        SmartDashboard.putNumber("AutoSOTM/TimeOfFlight", timeOfFlight);
        SmartDashboard.putNumber("AutoSOTM/pidOutput", pidOutput);

        return rotationalRate * MaxAngularRate;
    });
}));
        
        NamedCommands.registerCommand("Shoot", 
        new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SPINUP)));

        NamedCommands.registerCommand("ShooterOff", Commands.runOnce(() -> {
            PPHolonomicDriveController.clearRotationFeedbackOverride();
            superstructure.setDesiredState(Superstructure.SuperstructureState.IDLE);
        }));
        
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
        ); 
        */

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*
        opJoystick.povUp().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        opJoystick.povRight().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        opJoystick.povDown().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        opJoystick.povLeft().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */

        // Reset the field-centric heading on right stick press.
        //joystick.rightStick().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        drivetrain.registerTelemetry(logger::telemeterize);        

        /* Main driver Controller:
         * RT - Hold to spin up (and shoot hopefully) - relase to idle
         * RB - Shoot (dont use unless robot broken)
         * LT - Hold to Intake - release to idle
         * B - Reverse/outtake
         * Y - FAST SHOT (use if broken)
         * A - Hold to bump assist - release to idle
         * X - Robot off
         * Left Stick in - Reverse shi 
         */
        joystick.rightTrigger().onTrue(new InstantCommand(() -> {
            if (CommandSwerveDrivetrain.isInAllianceZone(drivetrain.getPose())) {
                superstructure.setDesiredState(Superstructure.SuperstructureState.SPINUP);
            } else {
                superstructure.setDesiredState(Superstructure.SuperstructureState.REVERSE);
            }
        }));

        /*
        joystick.rightBumper().onTrue(new SequentialCommandGroup(
            Commands.runOnce(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SHOOTONTHEMOVESPINUP)),
            Commands.waitSeconds(0.2),
            Commands.runOnce(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SHOOTONTHEMOVE))));
        */

        joystick.rightBumper().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.STATIONARYSHOT)));
        //joystick.rightBumper().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SHOOTONTHEMOVESPINUP)));
        
        
        joystick.y().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.REVERSE)));
        //joystick.y().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.FASTSHOT)));

        joystick.a().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState. BUMP)));

        joystick.x().onTrue(new InstantCommand(() -> IntakeSlideHandler.getInstance().setDesiredState(IntakeSlideState.REZEROIN))); 

        joystick.b().whileTrue(new SequentialCommandGroup(
            Commands.runOnce(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SPINUPFAST)),
            Commands.waitSeconds(0.2),
            Commands.runOnce(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.FASTSHOT)))
        );

        joystick.leftTrigger().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.INTAKESLOW)));
        joystick.leftBumper().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.INTAKE)));

        noButtonsHeld.onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.IDLE)));
        

       //joystick.a().onTrue(new InstantCommand(() -> superstructure.setDesiredState((Superstructure.SuperstructureState.TUNING))));
       //joystick.a().onFalse(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.OFF)));



        joystick.povUp().onTrue(Commands.runOnce(() -> shooter.changeShooterMult(0.05)));
        joystick.povDown().onTrue(Commands.runOnce(() -> shooter.changeShooterMult(-0.05)));
        

        joystick.povRight().whileTrue(IntakeSlide.manualDrive(() -> 0.67));
        joystick.povLeft().whileTrue(IntakeSlide.manualDrive(() -> -0.67));

        joystick.start().onTrue(Commands.runOnce(() -> shooter.resetShooterMult()));


        //CONTROLLER 2 / debug controller 
        /*
         *  RT - Shoot
         *  B - re-zero intake IN
         *  Y - overide alliance winner (on a switch)
         *  DPAD - shift hub by 0.1 M in direction (up is away, down is closer)
         *  A - Reset Shifted Hub to where it should be
         *  X - toggle hopperfullness
         */

         
        opJoystick.rightTrigger().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.STATIONARYSHOT)));
        opJoystick.y().onTrue(Commands.runOnce(() -> {
            var current = HubShiftUtil.getAllianceWinOverride();
            HubShiftUtil.setAllianceWinOverride(() -> Optional.of(current.orElse(true) == false));
        }));



        // Change Shooter Power
        
        opJoystick.povDown().onTrue(Commands.runOnce(() -> ShooterHandler.getInstance().adjustFastShot(-1))); 
        opJoystick.povUp().onTrue(Commands.runOnce(() -> ShooterHandler.getInstance().adjustFastShot(1))); //in RPs

        // Adjust Y offset
        opJoystick.povRight().onTrue(Commands.runOnce(() -> drivetrain.setHubOffset(0.0, 0.1)));
        opJoystick.povLeft().onTrue(Commands.runOnce(() -> drivetrain.setHubOffset(0.0, -0.1)));

        opJoystick.leftTrigger().onTrue(Commands.runOnce(() -> shooter.toggleShooterMult()));

        opJoystick.leftStick().onTrue(new InstantCommand(() -> drivetrain.ToggleSlowTele()));
 
        opJoystick.a().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.TUNING)));
        opJoystick.y().onTrue(Commands.runOnce(() -> drivetrain.resetHubOffset()));
        

        
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(HubShiftUtil::initialize));
        if(!RobotBase.isSimulation()){RobotModeTriggers.teleop().onTrue(Commands.runOnce(KickerSubsystem.getInstance()::setSlowCurrent));}
        RobotModeTriggers.teleop().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.IDLE)));
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(HubShiftUtil::initialize));
        RobotModeTriggers.autonomous().onTrue(new InstantCommand (() ->PPHolonomicDriveController.clearRotationFeedbackOverride()));
        RobotModeTriggers.autonomous().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.IDLE)));
        
        

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
