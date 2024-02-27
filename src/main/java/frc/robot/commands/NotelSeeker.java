// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Finder;
// import static frc.robot.Constants.*;
import frc.robot.subsystems.SwerveDrive;

public class NotelSeeker extends Command {
  /** Creates a new BallSeeker. */
  private SwerveDrive drivetrain;
  private Finder finder;
  private Collector collector;
  private int[] dize;

  public NotelSeeker(SwerveDrive drivetrain, Finder finder, Collector collector) {
    this.drivetrain = drivetrain;
    this.finder = finder;
    this.collector = collector;

    dize = new int[2];

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain , finder , collector);
  } //end BallSeeker

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.diableFieldOrientedDrive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dize = finder.findClosestTarget();
    if (dize[0] == 160) {
      drivetrain.podDriver(0, 0, 0.5);
    } //end if 
    else {
      collector.spinCollector(-1);
      drivetrain.podDriver(dize[0] / 100.0, 1, 0);
    } //end else
  } //end execute

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.spinCollector(0);
    drivetrain.podDriver(0, 0, 0);
    drivetrain.enableFieldOrientedDrive();
  } //ends end

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return collector.getNoteDetect();
  } //end isFinished
}