// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final double SHOOTER_RPM_MAX = 5800;
  private final double SHOOTER_FF_KS = .05;
  private final double SHOOTER_FF_KV = 12.0 / SHOOTER_RPM_MAX;
  private CANSparkMax tl;
  private CANSparkMax tf;
  private CANSparkMax bl;
  private CANSparkMax bf;

  private RelativeEncoder tEncoder;
  private RelativeEncoder bEncoder;

  private PIDController tControl;
  private PIDController bControl;
  private SimpleMotorFeedforward ff;
  
  public Shooter() {
  tl = new CANSparkMax(40, MotorType.kBrushless);
  tl.setInverted(false);
  tf = new CANSparkMax(41, MotorType.kBrushless);
  tf.follow(tl, !tl.getInverted());
  tEncoder = tl.getEncoder();
  tControl = new PIDController(.001, 0, 0);

  bl = new CANSparkMax(42, MotorType.kBrushless);
  bl.setInverted(false);
  bf = new CANSparkMax(43, MotorType.kBrushless);
  bf.follow(bl, !bl.getInverted());
  bEncoder = bl.getEncoder();
  bControl = new PIDController(.002, 0, 0);

  ff = new SimpleMotorFeedforward(SHOOTER_FF_KS, SHOOTER_FF_KV);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Top Flywheel", tEncoder.getVelocity()); 
    SmartDashboard.putNumber("Bottom Flywheel", bEncoder.getVelocity()); 
    
  }

  public void fireNoteManual(double speed) {
    speed *= .5;
    if (Math.abs(speed) > .01) {
      tl.setVoltage(speed * 12);
      bl.setVoltage(speed *  12);
    }
    else {
      tl.set(0);
      bl.set(0);
    }

  }
  public void fireNote(int rpm, boolean toAmp) {
    double ampMod;
    if (toAmp) {
      ampMod = 0.2;
    }
    else {
      ampMod = 1;
    }
    double setPoint = rpm;
    tl.setVoltage(ampMod * (tControl.calculate(tEncoder.getVelocity(), setPoint) + ff.calculate(setPoint)));
    bl.setVoltage((bControl.calculate(bEncoder.getVelocity(), setPoint) + ff.calculate(setPoint)));

  }
  public void stopShooter() {
    
  }
}
