// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class AutoCruise extends Command {
  private final SwerveDrive drivetrain;
  private double speed;
  private double direction;
  private double turn;
  private double distance;
  private double x1;
  private double y1;

  /** Creates a new AutoCruise. */
  public AutoCruise(double speed , double direction , double turn , double distance , SwerveDrive drivetrain) {
    this.drivetrain = drivetrain;
    this.speed = speed;
    this.direction = direction;
    this.turn = turn;
    this.distance = distance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double heading = direction * Math.PI / 180;
    y1 = speed * Math.cos(heading);
    x1 = speed * Math.sin(heading);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.podDriver(x1, y1, (turn - drivetrain.getYaw()) / 20.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.getAverageDistance() >= distance;
  }
}
