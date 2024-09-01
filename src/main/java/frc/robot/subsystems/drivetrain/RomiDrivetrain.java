// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class RomiDrivetrain extends BaseDrivetrain {


  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  private final RomiGyro gyro = new RomiGyro(); 

  private final DifferentialDriveOdometry odometry; 

  private final PIDController ctrlLeft = new PIDController(Constants.RomiDrivetrain.kWheelPIDLeft.kP, Constants.RomiDrivetrain.kWheelPIDLeft.kI, Constants.RomiDrivetrain.kWheelPIDLeft.kD);
  private final PIDController ctrlRight = new PIDController(Constants.RomiDrivetrain.kWheelPIDRight.kP, Constants.RomiDrivetrain.kWheelPIDRight.kI, Constants.RomiDrivetrain.kWheelPIDRight.kD);

  private SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism((volts) -> this.driveVoltage(volts.in(Units.Volts), volts.in(Units.Volts)), (log) -> {

      log.motor("drive-left")
        .voltage(Units.Volts.of(getLeftVolts()))
        .linearPosition(Units.Meters.of(getLeftDistanceMeters()))
        .linearVelocity(Units.MetersPerSecond.of(getLeftSpeed())); 

      log.motor("drive-right")
        .voltage(Units.Volts.of(getRightVolts()))
        .linearPosition(Units.Meters.of(getRightDistanceMeters()))
        .linearVelocity(Units.MetersPerSecond.of(getRightSpeed())); 
    }, this)
  );

  private Field2d m_field = new Field2d(); 

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse(Constants.RomiDrivetrain.distancePerPulse);
    m_rightEncoder.setDistancePerPulse(Constants.RomiDrivetrain.distancePerPulse);
    resetEncoders();

    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);

    this.odometry = new DifferentialDriveOdometry(getAngle(), 0, 0); 
  }

  public void arcadeDrive(double forwardMPS, double rotateRPS) {
    // m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
    SmartDashboard.putNumber("forward MPS", forwardMPS); 
    SmartDashboard.putNumber("rotate RPS", rotateRPS); 
    driveSpeeds(new ChassisSpeeds(forwardMPS, 0, rotateRPS));
  }

  public void driveSpeeds(ChassisSpeeds speeds) {
    // speeds.vxMetersPerSecond *= Constants.Drivetrain.scaleDownMultiplier; 
    DifferentialDriveWheelSpeeds wheelSpeeds = Constants.RomiDrivetrain.kinematics.toWheelSpeeds(speeds); 

    SmartDashboard.putNumber("Wheel Speeds Left", wheelSpeeds.leftMetersPerSecond); 
    SmartDashboard.putNumber("Wheel Speeds Right", wheelSpeeds.rightMetersPerSecond); 

    SmartDashboard.putNumber("Robot Chassis Forward", speeds.vxMetersPerSecond); 
    SmartDashboard.putNumber("Robot Chassis Turn", speeds.omegaRadiansPerSecond); 

    double leftFF = Constants.RomiDrivetrain.driveFFLeft.calculate(wheelSpeeds.leftMetersPerSecond);
    double rightFF = Constants.RomiDrivetrain.driveFFRight.calculate(wheelSpeeds.rightMetersPerSecond); 

    double leftOut = leftFF + ctrlLeft.calculate(getLeftSpeed(), wheelSpeeds.leftMetersPerSecond); 
    double rightOut = rightFF + ctrlRight.calculate(getRightSpeed(), wheelSpeeds.rightMetersPerSecond);
    
    SmartDashboard.putNumber("Left Out Volts", leftOut); 
    SmartDashboard.putNumber("Right Out Volts", leftOut); 

    driveVoltage(leftOut, rightOut);
  }

  public void driveVoltage(double leftVolts, double rightVolts) {
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(rightVolts);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceMeters() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeters() {
    return m_rightEncoder.getDistance();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(gyro.getAngle()); 
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters(); 
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(getAngle(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

  public double getLeftSpeed() {
    return m_leftEncoder.getRate(); 
  }

  public double getRightSpeed() {
    return m_rightEncoder.getRate(); 
  }

  public ChassisSpeeds getChassisSpeeds() {
    return Constants.RomiDrivetrain.kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed())); 

  }


  public double getLeftVolts() {
    return m_leftMotor.get() * RobotController.getInputVoltage(); 
  }

  public double getRightVolts() {
    return m_rightMotor.get() * RobotController.getInputVoltage(); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.odometry.update(getAngle(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance()); 

    this.m_field.setRobotPose(getPose());
    SmartDashboard.putData(this.m_field); 
    
    super.periodic();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  @Override
  public Field2d getField() {
    return this.m_field; 
  }
}
