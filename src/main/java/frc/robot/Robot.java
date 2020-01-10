/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a test program to easily test different values of RPM 
 * for the double wheel shooter on the 2020 robot.
 * 
 * RPM number boxes are sent to the smartdashboard for user input.
 * Actual RPM values from the encoder are display on the dashboard.
 * 
 * The neautral mode of motors can be set from the dashboard.
 * Change the value of the number box to 0 for brake and 1 for coast.
 * all other values will be ignored. 
 * 
 * Joystick buttons:
 * A - when pressed, toggles velocity PID control. target RPM is on the dashboard
 * X - enables joystick control of percent output. left Y is Top wheel. Right Y is Bottom wheel.
 * Start - stops motors and set the output to neutral
 * 
 */
public class Robot extends TimedRobot {
  // talon sensor velocity to RPM. 
  // (600 [100ms] / 1 [Minute]) * (1 [Rev] / 1023 [Native Encoder Ticks])
  private static final double kSensorVelocityToRPM = 600 / 1023; 

  private static final int kTopMotorID = 1;
  private static final int kBottomMotorID = 2;
  private static final int kJoystickPort = 0;
  
  private TalonSRX m_top_motor;
  private TalonSRX m_bottom_motor;
  private TalonSRXConfiguration m_talon_config;
  private XboxController m_joystick;

  private double m_top_RPM = 0;
  private double m_bottom_RPM = 0;
  private boolean m_isRunning = false;
  private int m_neutralMode = 1;

  @Override
  public void robotInit() {
    m_top_motor = new TalonSRX(kTopMotorID);
    m_bottom_motor = new TalonSRX(kBottomMotorID);
    m_joystick = new XboxController(kJoystickPort);
    
    // TODO: tune pid. k thanks.
    m_talon_config.voltageCompSaturation = 11;
    m_talon_config.continuousCurrentLimit = 25;
    m_talon_config.openloopRamp = 0.03; 
    m_talon_config.forwardSoftLimitEnable = false;
    m_talon_config.reverseSoftLimitEnable = false;
    m_talon_config.peakCurrentLimit = 50;
    m_talon_config.peakCurrentDuration = 10;
    m_talon_config.neutralDeadband = 0.03;
    m_talon_config.nominalOutputForward = 0;
    m_talon_config.nominalOutputReverse = 0;
    m_talon_config.peakOutputForward = 1;
    m_talon_config.peakOutputReverse = -1;
    m_talon_config.closedloopRamp = 0.03;
    m_talon_config.slot0.allowableClosedloopError = 0;
    m_talon_config.slot0.closedLoopPeakOutput = 1.0;
    m_talon_config.slot0.closedLoopPeriod = 2;
    m_talon_config.slot0.integralZone = 0;
    m_talon_config.slot0.kP = 0;
    m_talon_config.slot0.kI = 0;
    m_talon_config.slot0.kD = 0;
    m_talon_config.slot0.kF = 0.3;

    m_top_motor.configAllSettings(m_talon_config);
    m_top_motor.setNeutralMode(NeutralMode.Coast);
    m_bottom_motor.configAllSettings(m_talon_config);
    m_bottom_motor.setNeutralMode(NeutralMode.Coast);

    SmartDashboard.putNumber("Top RPM", m_top_RPM);
    SmartDashboard.putNumber("Bot RPM", m_bottom_RPM);
    SmartDashboard.putNumber("NeutralMode (0 brake / 1 coast)", (double) m_neutralMode);
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Top RPM", m_top_motor.getSelectedSensorVelocity() * kSensorVelocityToRPM);
    SmartDashboard.putNumber("Bot RPM", m_bottom_motor.getSelectedSensorVelocity() * kSensorVelocityToRPM);
    m_top_RPM = SmartDashboard.getNumber("Top RPM", 0);
    m_bottom_RPM = SmartDashboard.getNumber("Bot RPM", 0);
  }
  

  @Override
  public void teleopPeriodic() {
    // A button will toggle the control loop off and on
    if (m_joystick.getAButtonPressed()) {
      m_isRunning = m_isRunning ? !m_isRunning : m_isRunning;
    }

    // start button = stop / panic button
    if (m_joystick.getStartButton()) {
      m_isRunning = false;
      stopMotors();
    }
    
    // check if SM value is different than current and then update motors if necessary
    int smartDashboard_neutralMode = (int) SmartDashboard.getNumber("NeutralMode (0 brake / 1 coast)", 0);
    if (smartDashboard_neutralMode != m_neutralMode) {
      switch (smartDashboard_neutralMode) {
        case 0:
          m_top_motor.setNeutralMode(NeutralMode.Brake);
          m_bottom_motor.setNeutralMode(NeutralMode.Brake);
          m_neutralMode = smartDashboard_neutralMode;
          break;
        case 1:
          m_top_motor.setNeutralMode(NeutralMode.Coast);
          m_bottom_motor.setNeutralMode(NeutralMode.Coast);
          m_neutralMode = smartDashboard_neutralMode;
          break;
        default:
          SmartDashboard.putNumber("NeutralMode (0 brake / 1 coast)", m_neutralMode);
        break;
      }
    }

    if (m_joystick.getXButton()) {
      // enable joystick percent output
      m_top_motor.set(ControlMode.PercentOutput, m_joystick.getY(Hand.kLeft) * -1 );
      m_bottom_motor.set(ControlMode.PercentOutput, m_joystick.getY(Hand.kRight) * -1 );
    } else if (m_isRunning) {
      // set the velocity to the RPM grabbed from dashboard
      m_top_motor.set(ControlMode.Velocity, m_top_RPM / kSensorVelocityToRPM );
      m_bottom_motor.set(ControlMode.Velocity, m_bottom_RPM / kSensorVelocityToRPM );
    } else {
      stopMotors();
    }
  }

  @Override
  public void disabledInit() {
    super.disabledInit();
    stopMotors();
  }

  private void stopMotors() {
    m_top_motor.neutralOutput();
    m_bottom_motor.neutralOutput();
  }
}
