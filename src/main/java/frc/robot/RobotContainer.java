// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AimTele;
import frc.robot.commands.AutoCruise;
import frc.robot.commands.AutoPickup;
import frc.robot.commands.FireNoteAuto;
import frc.robot.commands.FireNoteCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;
//import frc.robot.commands.Autos;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Targeting;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 * 
 * Praj Box Documentation
 * these are the button Id's for each switch
 * safety switch 1
 * right 3way up 2
 * right 3way down 3
 * button 4
 * left 3way up 5
 * left 3way down 6
 * white switch 7
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrive drive = new SwerveDrive();
  private final Shooter shooter = new Shooter();
  private final Collector collector = new Collector();
  private final Climber climber = new Climber();
  private final Targeting targeting = new Targeting();

  private final Command leave = new AutoCruise(1, 0, 0, 3.5, drive);
  private final SequentialCommandGroup shootLeave = new SequentialCommandGroup(
    new FireNoteAuto(shooter, collector , targeting),
    new AutoCruise(1, 0, 0, 3.4, drive)
  );
  private final SequentialCommandGroup twoNoteDR = new SequentialCommandGroup(
    new FireNoteAuto(shooter, collector , targeting),
    Commands.deadline(new AutoPickup(collector) , new AutoCruise(1, 0, 0, 6.6, drive))
  );


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController xbox =
      new CommandXboxController(0);
  private final CommandXboxController xboxOp =
      new CommandXboxController(1);
  private final Joystick prajBox =
      new Joystick(2);
  private Trigger enableClimber = new JoystickButton(prajBox, 1);
  private Trigger overrideCollector = new JoystickButton(prajBox, 7);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //regular Drive
    drive.setDefaultCommand(Commands.run(
      () -> drive.podDriver(-xbox.getLeftX(), -xbox.getLeftY(), -xbox.getRightX()),
      drive
    ));

    //collector automation
    collector.setDefaultCommand(Commands.run(
      () -> collector.holdCollector(xboxOp.a().getAsBoolean() , xboxOp.y().getAsBoolean()),
      collector
      ));

    configureBindings();
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
    //reset imu, point shooter at driver wall
    xbox.back().onTrue(Commands.runOnce(drive :: resetYaw , drive));
    //turbo
    xbox.rightTrigger()
      .onTrue(Commands.runOnce(drive :: enableTurbo, drive))
      .onFalse(Commands.runOnce(drive :: disableTurbo, drive));
    //target speaker
    xbox.rightBumper().whileTrue(new AimTele(0, drive, targeting));
    //target amp
    xbox.leftBumper().whileTrue((new AimTele(2, drive, targeting)));

    //shoot for speaker
    xboxOp.rightBumper().onTrue(new FireNoteCommand(shooter , targeting));
    //shoot for amp    
    xboxOp.start().onTrue(Commands.run(shooter :: fireNoteAmp , shooter));
    //turn shooter off
    xboxOp.leftBumper().onTrue(Commands.runOnce(shooter :: stopShooter, shooter));

    //white prajbox switch on overrides collector automation
    overrideCollector.whileTrue(Commands.run(
      () -> collector.manualOverride(-xboxOp.getRightY() , -xboxOp.getLeftY()),
      collector
      ));
    //collector out
    xboxOp.rightTrigger(.5).whileTrue(Commands.run(collector :: fire, collector));
    //collector in
    xboxOp.leftTrigger(.5).whileTrue(Commands.run(collector :: intake, collector));    

    //prajbox safety switch on activates climbers on sticks, disables collector
    //hold button to reverse
    enableClimber.whileTrue(Commands.deadline(
      Commands.run(() -> climber.runClimbers(-xboxOp.getLeftY(), -xboxOp.getRightY(), 
        true, prajBox),
        climber),
      Commands.run(collector :: off, collector)
      ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
