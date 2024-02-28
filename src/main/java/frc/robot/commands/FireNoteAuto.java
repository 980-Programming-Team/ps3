// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Targeting;

public class FireNoteAuto extends Command {
  private Shooter shooter;
  private Collector collector;
  private Targeting targeting;
  private double rpmHigh;
  private double rpmLow;
  private int timer;
  /** Creates a new FireNote. */
  public FireNoteAuto(Shooter shooter , Collector collector , Targeting targeting) {
    this.shooter = shooter;
    this.collector = collector;
    this.targeting = targeting;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter , collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collector.holdCollector(false, true);
    timer = 3;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO autoranging goes here
    shooter.fireNote(2000, 2000);  
    if(shooter.isReady() && collector.isUp()){
      collector.fire();
      timer--;
    }  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    collector.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer == 0){
      return true;
    }
    else{
      return false;
    }
 }
}
