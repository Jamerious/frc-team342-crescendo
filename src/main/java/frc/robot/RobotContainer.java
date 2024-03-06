// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Drive.DriveDistance;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.TimedDrive;
import frc.robot.commands.Autos.Autos;
import frc.robot.commands.Autos.Autos.*;
import frc.robot.commands.Drive.DriveWithJoystick;
import frc.robot.commands.Outtake.OuttakeNote;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;

import javax.swing.text.WrappedPlainView;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.Climb;
import frc.robot.commands.Load;
import frc.robot.commands.MoveWristPercent;
import frc.robot.commands.MoveWristToPosition;
import frc.robot.commands.Drive.DriveWithJoystick;
import edu.wpi.first.wpilibj.XboxController;

//import static frc.robot.Constants.IntakeConstants.feedShooterSpeed;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.SwerveDrive;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final JoystickButton xButton;
  // private final JoystickButton aButton;
  
  private SwerveDrive swerve;
  // private Outtake outtake;
  
  private XboxController driver;
  private XboxController operator;

  private DriveWithJoystick driveWithJoystick;
  private DriveDistance driveDistance;
  private OuttakeNote outtakeNote;
  private RotateToAngle rotate90;
  private MoveWristToPosition moveWristDown;
  private MoveWristToPosition moveWristUp;
  private MoveWristToPosition moveWristAmp;
  private Autos shootmiddle;

  // private MoveWristToPosition moveWrist;


  private Climb climb;

  private Outtake shootVelocity;

  private Load load;
  private Outtake outtake;
  private XboxController joy;

  //private JoystickButton goToZeroBtn;
  private JoystickButton rotateToAngleButton;
  private JoystickButton toggleFieldOrientedBtn;
  private JoystickButton toggleSlowModeBtn;
  private JoystickButton outtakeNoteBtn;
  private JoystickButton driveDistanceButton;
  private JoystickButton rotateBTN;
  private JoystickButton wristButton;
  private JoystickButton intakeBtn;

  private SendableChooser<Command> autoChooser;
  private JoystickButton climbButton;

  //private POVButton wristDownBtn;
  //private POVButton wristUpBtn;
  //private POVButton wristRightBtn;
  //private POVButton wristLeftBtn;

  private Intake intake;
  private Wrist wrist;
  private JoystickButton loadButton;
  private Elevator elevator;

  private MoveWristPercent moveWristPercent;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Subsystems
    intake = new Intake();
    outtake = new Outtake();
    load = new Load(outtake, intake);
    wrist = new Wrist();
    
    swerve = new SwerveDrive();

    joy = new XboxController(0);
    driveWithJoystick = new DriveWithJoystick(swerve, joy);
    // outtakeNote = new OuttakeNote(0.5, outtake);
    driveDistance = new DriveDistance(1, 5, swerve);

    //driveWithJoystick = new DriveWithJoystick(swerve, joy, swerve.getFieldOriented());
    rotateToAngleButton = new JoystickButton(joy, XboxController.Button.kB.value);
    
    
    swerve.setDefaultCommand(driveWithJoystick);
    toggleFieldOrientedBtn = new JoystickButton(joy, XboxController.Button.kA.value);
    toggleSlowModeBtn = new JoystickButton(joy, XboxController.Button.kX.value);
    // outtakeNoteBtn = new JoystickButton(joy, XboxController.Button.kB.value);
    driveDistanceButton = new JoystickButton(joy, XboxController.Button.kY.value);

    elevator = new Elevator();
    // swerve = new SwerveDrive();

    // Xbox Controllers
    driver = new XboxController(0);
    operator = new XboxController(1);

    // Intake Buttons
    xButton = new JoystickButton(operator, XboxController.Button.kX.value);
    // aButton = new JoystickButton(joy, XboxController.Button.kA.value);
    wristButton = new JoystickButton(operator, XboxController.Button.kY.value);
    loadButton = new JoystickButton(operator, XboxController.Button.kB.value);
    intakeBtn = new JoystickButton(operator, XboxController.Button.kA.value);
    rotateBTN = new JoystickButton(joy, XboxController.Button.kB.value);
    
    driveWithJoystick = new DriveWithJoystick(swerve, driver);

    //moveWristDown = new MoveWristToPosition(intake, IntakeConstants.LOW_WRIST_POS);
   // moveWristUp = new MoveWristToPosition(intake, IntakeConstants.HIGH_WRIST_POS);
   // moveWristAmp = new MoveWristToPosition(intake, IntakeConstants.AMP_POS);
    loadButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    intakeBtn = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    outtakeNoteBtn = new JoystickButton(operator, XboxController.Button.kA.value);

    // Climb Buttons
    climbButton = new JoystickButton(operator, XboxController.Button.kStart.value);

    // Driver-assisted Buttons
    //wristDownBtn = new POVButton(operator, 180);
    //wristUpBtn = new POVButton(operator, 0);
    //wristRightBtn = new POVButton(operator, 270);
    //wristLeftBtn = new POVButton(operator, 90);

    // Toggle Buttons
    toggleFieldOrientedBtn = new JoystickButton(driver, XboxController.Button.kA.value);
    toggleSlowModeBtn = new JoystickButton(driver, XboxController.Button.kX.value);
    
    driveWithJoystick = new DriveWithJoystick(swerve, driver);

    load = new Load(outtake, intake);
    climb = new Climb(elevator, operator);

    moveWristDown = new MoveWristToPosition(wrist, intake, IntakeConstants.LOW_WRIST_POS);
    moveWristUp = new MoveWristToPosition(wrist, intake, IntakeConstants.HIGH_WRIST_POS);
    moveWristAmp = new MoveWristToPosition(wrist, intake, IntakeConstants.AMP_POS);

   // wristDownIntake = new SequentialCommandGroup(moveWristDown, intake.spinIntake().until(() -> !intake.getIntakeSensor()));

   // moveWristPercent = new MoveWristPercent(operator, intake);
    intake.setDefaultCommand(moveWristPercent);

    outtakeNoteBtn = new JoystickButton(operator, XboxController.Button.kA.value);
    
   // wristDownIntake = new SequentialCommandGroup(moveWristDown, intake.spinIntake().until(() -> !intake.getIntakeSensor()));
    moveWristPercent = new MoveWristPercent(operator, wrist);

    wrist.setDefaultCommand(moveWristPercent);
    swerve.setDefaultCommand(driveWithJoystick);
    elevator.setDefaultCommand(climb);

   // SmartDashboard.putData(swerve);
   SmartDashboard.putData(swerve);
   SmartDashboard.putData(outtake);
   SmartDashboard.putData(intake);
   SmartDashboard.putData(wrist);

    configureBindings();


    autoChooser = new SendableChooser<>();

    autoChooser.addOption("Middle Auto 2 Piece", Autos.MiddleShoot(swerve, outtake, intake, wrist));

    autoChooser.addOption("Left Side Speaker 2 piece", Autos.LEftAuto(swerve, outtake, intake, wrist));
    
    autoChooser.addOption("Right Side Speaker 2 Piece", Autos.RightAuto(swerve, outtake, intake, wrist, new ChassisSpeeds(1,0,0)));

    autoChooser.addOption("Do nothing", Autos.DoNothing());

    autoChooser.addOption("Shoot and Scoot", Autos.shootAndScoot(swerve,outtake,intake, new ChassisSpeeds(1,0,0)));

    SmartDashboard.putData(autoChooser);

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
    toggleFieldOrientedBtn.whileTrue(swerve.toggleFieldOriented());
    toggleSlowModeBtn.whileTrue(swerve.toggleSlowMode());
    // outtakeNoteBtn.whileTrue(outtakeNote);
    //goToZeroBtn.whileTrue(swerve.goToZero());
    
   xButton.whileTrue(intake.spinIntake());
  //  aButton.whileTrue(intake.getSensors());
  //  wristButton.whileTrue(moveWrist);
   loadButton.whileTrue(load);
   intakeBtn.whileTrue(intake.spinIntake());
    xButton.whileTrue(intake.outtake()); // X
    loadButton.whileTrue(load); // B
    intakeBtn.whileTrue(intake.spinIntake()); // A
   // wristDownBtn.onTrue(wristDownIntake); // Down on D-Pad
    //wristUpBtn.onTrue(moveWristUp); // Up on D-Pad
    //wristLeftBtn.onTrue(moveWristAmp); // Right on D-Pad
    //wristRightBtn.onTrue(moveWristAmp); // Left on D-Pad

    climbButton.whileTrue(elevator.toggleClimbMode());

    toggleFieldOrientedBtn.whileTrue(swerve.toggleFieldOriented());
    toggleSlowModeBtn.whileTrue(swerve.toggleSlowMode());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return autoChooser.getSelected();




  }
}