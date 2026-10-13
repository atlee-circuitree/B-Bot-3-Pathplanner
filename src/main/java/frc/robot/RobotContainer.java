// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ManualDeplolyWithinReason;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.KickerUper;
import frc.robot.commands.ManualDeploy;
import frc.robot.commands.MidDeploy;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunOuttake;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootInterrupt;
import frc.robot.commands.StopIntake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.KickUp;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

//import java.lang.invoke.ClassSpecializer.SpeciesData;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private Intake intake;
  private Shooter shoot;
  private KickUp kickUp;

  private static final Field2d field = new Field2d();
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(field);

  private final double MaxSpeed = Constants.Drive.MAX_SPEED_MPS / Constants.Drive.SPEED_DIVISOR;
  private final double MaxAngularRate = Constants.Drive.MAX_ANGULAR_RATE_RPS;


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.Drive.DEADBAND_PERCENT).withRotationalDeadband(MaxAngularRate * Constants.Drive.DEADBAND_PERCENT)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController Player1 = new CommandXboxController(0);
  private final CommandXboxController Player2 = new CommandXboxController(1);

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
 // private final ShootSubsystem Shooter = new ShooterSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SendableChooser<Command> autochooser;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


    
    //SmartDashboard.putData(autochooser);

    // Configure the trigger bindings

    intake = new Intake();
    shoot = new Shooter();
    kickUp = new KickUp();
    configureBindings();
     mapEventsToCommands();
    
    autochooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("auto chooser", autochooser);
  }


  public void seedFieldOrient() {
        drivetrain.seedFieldCentric();


    }

    public double getLeftY()
    {
        double player1 = Player1.getLeftY();
        double player2 = Player2.getLeftY();

        double max = Math.abs(player1) > Math.abs(player2) ? player1 : player2;
        return max;
    }

    public double getLeftX()
    {
        double player1 = Player1.getLeftX();
        double player2 = Player2.getLeftX();

        double max = Math.abs(player1) > Math.abs(player2) ? player1 : player2;
        return max;
    }

    public double getRightX()
    {
        double player1 = Player1.getRightX();
        double player2 = Player2.getRightX();

        double max = Math.abs(player1) > Math.abs(player2) ? player1 : player2;
        return max;
        }
    
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    configureDrivetrain();
 drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(-(getLeftY()) * ((Constants.Drive.MAX_SPEED_MPS) / Constants.Drive.SPEED_DIVISOR)) // Drive forward with negative Y (forward)
                    .withVelocityY(-(getLeftX()) * ((Constants.Drive.MAX_SPEED_MPS) / Constants.Drive.SPEED_DIVISOR)) // Drive left with negative X (left)
                    .withRotationalRate(-(getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
   
    SmartDashboard.putNumber("shoot speed", 0);

  // shoot close
   // Player1.x().whileTrue(new ParallelCommandGroup(
    //  new Shoot(shoot, Constants.Shooter.ShootSpeedClose),
      // new KickerUper(kickUp, shoot)
     // new KickerUper(kickUp, shoot)

      //new RunOuttake(intake) 
   // )); 

  // shoot far
  //Player1.rightTrigger().whileTrue(new ParallelCommandGroup(
     // new Shoot(shoot, Constants.Shooter.ShootSpeedFar),
     // new KickerUper(kickUp, shoot)
     // new RunOuttake(intake) 
    //));
  
  //commenting out the following for testing.
  /* 
  Player1.leftBumper().whileTrue(new ParallelCommandGroup(
    new Shoot(shoot, Constants.Shooter.ShootSpeedClose ), 
    new KickerUper(kickUp, shoot)
    //new RunOuttake(intake) 
    ));
    */
    Player1.rightTrigger().whileTrue(
    new SequentialCommandGroup(
        // Step 1: Un-jam/back up the note for 0.25 seconds
        new ParallelCommandGroup(
            shoot.run(() -> shoot.runAtPower(-0.1)),       // Run shooter in reverse
            kickUp.run(() -> kickUp.reverse())     // Or a reverse command for kickUp/intake
        ).withTimeout(0.25),
        new WaitCommand(0.5),
        // Step 2: Normal shooting sequence
        new ParallelCommandGroup(
            new Shoot(shoot, Constants.Shooter.ShootSpeedFar),
            new KickerUper(kickUp, shoot)
        )
    ).finallyDo(() -> {
        // ULTIMATE SAFETY CATCH:
        // When the bumper is released, forcefully shut off all motors
        shoot.stopShooter();
        kickUp.stop();
    })
);
  Player1.x().whileTrue(
 new SequentialCommandGroup(
        // Step 1: Un-jam/back up the note for 0.25 seconds
        new ParallelCommandGroup(
            shoot.run(() -> shoot.runAtPower(-0.1)),       // Run shooter in reverse
            kickUp.run(() -> kickUp.reverse())     // Or a reverse command for kickUp/intake
        ).withTimeout(0.25),
        new WaitCommand(0.5),
        // Step 2: Normal shooting sequence
        new ParallelCommandGroup(
            new Shoot(shoot, Constants.Shooter.ShootSpeedClose),
            new KickerUper(kickUp, shoot)
        )
    ).finallyDo(() -> {
        // ULTIMATE SAFETY CATCH:
        // When the bumper is released, forcefully shut off all motors
        shoot.stopShooter();
        kickUp.stop();
    })
);

    Player1.povUp().whileTrue(new ParallelCommandGroup(
    new Shoot(shoot, Constants.Shooter.ShootSpeedFar)
    //new KickerUper(kickUp, shoot, -0.1)
     //new RunOuttake(intake) 
    ));

    // Moves the arm downward at 30% speed while POV Left is held, and stops when released
    Player1.povLeft().whileTrue(new ManualDeplolyWithinReason(intake, -Constants.Intake.DeploySpeed));
    Player1.povRight().whileTrue(new ManualDeplolyWithinReason(intake, Constants.Intake.DeploySpeed));
    Player1.leftBumper().whileTrue(new MidDeploy(intake, -Constants.Intake.DeploySpeed));


    
    //(new ManualDeploy(intake, -0.3, "deploy")
    
    
    
    
    Player1.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    //Player1.pov Up().whileTrue(new Shoot(shoot, -.1));
    Player1.a().whileTrue(new ManualDeploy(intake, Constants.Intake.DeploySpeed));
    Player1.b().whileTrue(new ManualDeploy(intake, -Constants.Intake.DeploySpeed));
    Player1.povDown().whileTrue(new RunIntake(intake));//outtake
    Player1.leftTrigger().whileTrue(new RunOuttake(intake));//intake
    //Player1.povDown().whileTrue(new IntakeCommand(intake, 50, "deploy"));

    //Player1.povUp().whileTrue(new Shoot(shoot, 0.65));
  
    Player1.y().whileTrue(new KickerUper(kickUp, shoot));
  
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }
 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return autochooser.getSelected();
    return autochooser.getSelected();
  }

 private void configureDrivetrain() {
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void mapEventsToCommands()
    {
        // replace null with command instance
        // do this for all commands
        NamedCommands.registerCommand("shoot close",new SequentialCommandGroup(
        // Step 1: Un-jam/back up the note for 0.25 seconds
        new ParallelCommandGroup(
            shoot.run(() -> shoot.runAtPower(-0.2)),       // Run shooter in reverse
            kickUp.run(() -> kickUp.reverse())     // Or a reverse command for kickUp/intake
        ).withTimeout(1.0),
        new WaitCommand(0.5),
        // Step 2: Normal shooting sequence
        new ParallelCommandGroup(
            new Shoot(shoot, Constants.Shooter.ShootSpeedClose),
            new KickerUper(kickUp, shoot)
        )
    ).finallyDo(() -> {
        // ULTIMATE SAFETY CATCH:
        // When the bumper is released, forcefully shut off all motors
        shoot.stopShooter();
        kickUp.stop();
    }));
    NamedCommands.registerCommand("shoot far",new SequentialCommandGroup(
        // Step 1: Un-jam/back up the note for 0.25 seconds
        new ParallelCommandGroup(
            shoot.run(() -> shoot.runAtPower(-0.4)),       // Run shooter in reverse
            kickUp.run(() -> kickUp.reverse())     // Or a reverse command for kickUp/intake
        ).withTimeout(1.5),
        new WaitCommand(0.5),
        // Step 2: Normal shooting sequence
        new ParallelCommandGroup(
            new Shoot(shoot, Constants.Shooter.ShootSpeedFar),
            new KickerUper(kickUp, shoot)
        )
    ).finallyDo(() -> {
        // ULTIMATE SAFETY CATCH:
        // When the bumper is released, forcefully shut off all motors
        shoot.stopShooter();
        kickUp.stop();
    }));


        NamedCommands.registerCommand("deploy", new ManualDeploy(intake, Constants.Intake.DeploySpeed).withTimeout(.5));
        NamedCommands.registerCommand("intake shoot", new RunOuttake(intake).withTimeout(7));
        NamedCommands.registerCommand("intake", new RunOuttake(intake).withTimeout(2));

    }

}
