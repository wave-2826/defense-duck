// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/** Represents a differential drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = 0.381 * 2; // meters

  private final WPI_TalonSRX m_leftLeader = new WPI_TalonSRX(1);
  private final WPI_TalonSRX m_leftFollower = new WPI_TalonSRX(2);
  private final WPI_TalonSRX m_rightLeader = new WPI_TalonSRX(3);
  private final WPI_TalonSRX m_rightFollower = new WPI_TalonSRX(4);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;

  // Gains are for CIM motors - must be tuned for your specific robot!
  // kS (static gain): ~1.5 for CIM motors to overcome static friction
  // kV (velocity gain): ~3.0 for drivetrain with CIM motors
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1.5, 3.0);

  /**
   * Constructs a differential drive object. Sets up CIM motors via TalonSRX for CAN control.
   */
  public Drivetrain() {
    m_gyro.reset();

    // Configure TalonSRX controllers for CIM motors
    // Set neutral mode to brake for better stopping power
    m_leftLeader.setNeutralMode(NeutralMode.Brake);
    m_leftFollower.setNeutralMode(NeutralMode.Brake);
    m_rightLeader.setNeutralMode(NeutralMode.Brake);
    m_rightFollower.setNeutralMode(NeutralMode.Brake);

    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightLeader.setInverted(true);
    m_rightFollower.setInverted(true);

    m_odometry =
        new DifferentialDriveOdometry(
            m_gyro.getRotation2d(), 0.0, 0.0);
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    // Without encoders, we rely on TalonSRX native feedback or open-loop voltage control
    // Set voltage to motor controllers based on feedforward calculations
    m_leftLeader.setVoltage(leftFeedforward);
    m_rightLeader.setVoltage(rightFeedforward);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(), 0.0, 0.0);
  }
}
