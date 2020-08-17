/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a sample program which uses joystick buttons to control a relay. A Relay (generally a
 * spike) has two outputs, each of which can be at either 0V or 12V and so can be used for actions
 * such as turning a motor off, full forwards, or full reverse, and is generally used on the
 * compressor. This program uses two buttons on a joystick and each button corresponds to one
 * output; pressing the button sets the output to 12V and releasing sets it to 0V.
 */

public class Robot extends TimedRobot {
  private final Joystick m_stick = new Joystick(0);
  private final Encoder m_encoder= new Encoder(7, 8, false, CounterBase.EncodingType.k4X);
  private final TalonSRX arm = new TalonSRX(0);
  private final int  db = 15;

  public void robotInit() {
    m_encoder.setSamplesToAverage(5);
    m_encoder.setDistancePerPulse(1.0 / 1340 * 360);
    m_encoder.setMinRate(1.0);
  }
 
  @Override
  public void teleopInit() {
    m_encoder.reset();
  }
  @Override
  public void teleopPeriodic() {
    if(m_stick.getRawButton(1) && !(m_encoder.getDistance() < (90 + db) && m_encoder.getDistance() > (90 - db))) {
      arm.set(ControlMode.PercentOutput, 0.5);
      SmartDashboard.putNumber("Encoder Distance", m_encoder.getDistance());
      SmartDashboard.putNumber("Encoder Rate", m_encoder.getRate());
    }
    else if(m_stick.getRawButton(2) && !(m_encoder.getDistance() < (180 + db) && m_encoder.getDistance() > (180 - db))){
      arm.set(ControlMode.PercentOutput, 0.5);
      SmartDashboard.putNumber("Encoder Distance", m_encoder.getDistance());
      SmartDashboard.putNumber("Encoder Rate", m_encoder.getRate());
    }
    else if(m_stick.getRawButton(3) && !(m_encoder.getDistance() < (270 + db) && m_encoder.getDistance() > (270 - db))){
      arm.set(ControlMode.PercentOutput, 0.5);
      SmartDashboard.putNumber("Encoder Distance", m_encoder.getDistance());
      SmartDashboard.putNumber("Encoder Rate", m_encoder.getRate());
    }
    else if(m_stick.getRawButton(4) && !(m_encoder.getDistance() < (360 + db) && m_encoder.getDistance() > (360 - db))){
      arm.set(ControlMode.PercentOutput, 0.5);
      SmartDashboard.putNumber("Encoder Distance", m_encoder.getDistance());
      SmartDashboard.putNumber("Encoder Rate", m_encoder.getRate());
    }
    else {
      arm.set(ControlMode.PercentOutput, 0);
    }

    if(m_encoder.getDistance() > 360) m_encoder.reset();
  }
}
