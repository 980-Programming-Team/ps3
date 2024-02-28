// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Targeting;

public class AimTele extends Command {
  private SwerveDrive drivetrain;
  private Targeting targeting;
  private int whichTarget;//0 speaker, 1 altSpeaker, 2 amp, 3 source left, 4 stage back

  /** Creates a new Aim. */
  public AimTele(int whichTarget , SwerveDrive drivetrain, Targeting targeting) {
    this.whichTarget = whichTarget;
    this.drivetrain = drivetrain;
    this.targeting = targeting;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.disableFieldOrientedDrive();
    targeting.changeTag(whichTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(whichTarget == 0){
      drivetrain.podDriver(0, 0, (targeting.getX())/20);
    }
    else if(whichTarget == 2){
      if(targeting.getSide()){
        drivetrain.podDriver((targeting.getX())/10, 0, (90 - drivetrain.getYaw()) / 20.0);
      }
      else{
        drivetrain.podDriver((targeting.getX())/10, 0, (-90 - drivetrain.getYaw()) / 20.0);
      }
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.enableFieldOrientedDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
