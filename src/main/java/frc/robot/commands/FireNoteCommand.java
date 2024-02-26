// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;

public class FireNoteCommand extends Command {
  private final Shooter shooter;
  private final Collector collector;
  private final DoubleSupplier fire;
  private boolean toAmp;
  /** Creates a new FireNote. */
  public FireNoteCommand(DoubleSupplier fire, boolean toAmp , Shooter shooter , Collector collector) {
    this.shooter = shooter;
    this.collector = collector;
    this.fire = fire;
    this.toAmp = toAmp;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter , collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.fireNote(2000, toAmp);
    if(fire.getAsDouble() > .5){
      collector.spinCollector(-1);
    }
    else{
      collector.spinCollector(0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
