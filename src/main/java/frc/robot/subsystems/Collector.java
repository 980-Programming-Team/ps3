// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {
  private CANSparkMax deploy;
  private CANSparkMax collect;

  private RelativeEncoder deployEncoder;
  private final double DOWN_POSITION = .25;//TODO change to how many rotations it actually takes to deploy
  private final double UP_POSITION = .1;//TODO tune this so the motor is not trying to get to a position it can't reach mechanically
  private boolean down;

  private DigitalInput primaryNoteDetect;
  private DigitalInput backupNoteDetect;


  /** Creates a new Collector. */
  public Collector() {
    deploy = new CANSparkMax(44, MotorType.kBrushless);
    deployEncoder = deploy.getEncoder();
    deployEncoder.setPositionConversionFactor(1 / 100.0);
    collect = new CANSparkMax(46, MotorType.kBrushless);

    down = false;

    primaryNoteDetect = new DigitalInput(0);
    backupNoteDetect = new DigitalInput(1);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Collector Position", deployEncoder.getPosition());
    // This method will be called once per scheduler run
  }

  public void manualOverride(double speed , double dSpeed){
    deploy.set(dSpeed);
    spinCollector(speed);
  }

  public void deployCollector(){
    down = true;
  }

  public void retractCollector(){
    down = false;
  }
  public boolean isDown(){
    return down;
  }

  public void holdCollector(){
    if(down && deployEncoder.getPosition() < DOWN_POSITION){
      deploy.set(.5);//TODO figure out speed currently .5
    }
    else if(!down && deployEncoder.getPosition() > UP_POSITION){
      if(deployEncoder.getPosition() / DOWN_POSITION > .7){
        deploy.set(-1);//run full until 3/4 up TODO tune where to start cutting power
      }
      else{
        deploy.set(-deployEncoder.getPosition() / DOWN_POSITION);//end with proportional control, TODO may need a multiplier
      }
    }
    else{
      deploy.set(0);
    }//end position holder
  }

  public void spinCollector(double speed){
    holdCollector();
    if(Math.abs(speed) > .1){
      if(down){
        collect.set(speed);
      }
      else{
        collect.set(0);
      }
    }
    else{
      collect.set(0);
    }
  }

  public boolean getNoteDetect(){
    return primaryNoteDetect.get() || backupNoteDetect.get();
  }


}
