package frc.robot.subsystems.drivetrain;

import java.lang.invoke.ConstantCallSite;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class XRPDrivetrain extends BaseDrivetrain {

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  private final XRPGyro gyro = new XRPGyro(); 

  private final DifferentialDriveOdometry odometry; 

  private final PIDController ctrlLeft = new PIDController(Constants.XRPDrivetrain.kWheelPIDLeft.kP, Constants.XRPDrivetrain.kWheelPIDLeft.kI, Constants.XRPDrivetrain.kWheelPIDLeft.kD);
  private final PIDController ctrlRight = new PIDController(Constants.XRPDrivetrain.kWheelPIDRight.kP, Constants.XRPDrivetrain.kWheelPIDRight.kI, Constants.XRPDrivetrain.kWheelPIDRight.kD);

  private SysIdRoutine routine; 

  private Field2d m_field = new Field2d(); 

  // TODO: WPILib 2025.0.0 should relieve the need to manually estimate velocity
  private double leftDist = 0; 
  private double rightDist = 0; 

  private double curLeftDist = 0; 
  private double curRightDist = 0; 

  /** Creates a new XRPDrivetrain. */
  public XRPDrivetrain() {

    System.out.println(this.m_leftEncoder.getPeriod());

    this.routine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism((volts) -> this.driveVoltage(volts.in(Units.Volts), volts.in(Units.Volts)), (log) -> {
  
        System.out.println(getLeftSpeed()); 

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

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse(Constants.XRPDrivetrain.distancePerPulse);
    m_rightEncoder.setDistancePerPulse(Constants.XRPDrivetrain.distancePerPulse);
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
    DifferentialDriveWheelSpeeds wheelSpeeds = Constants.XRPDrivetrain.kinematics.toWheelSpeeds(speeds); 

    SmartDashboard.putNumber("Wheel Speeds Left", wheelSpeeds.leftMetersPerSecond); 
    SmartDashboard.putNumber("Wheel Speeds Right", wheelSpeeds.rightMetersPerSecond); 

    SmartDashboard.putNumber("Robot Chassis Forward", speeds.vxMetersPerSecond); 
    SmartDashboard.putNumber("Robot Chassis Turn", speeds.omegaRadiansPerSecond); 

    double leftFF = Constants.XRPDrivetrain.driveFFLeft.calculate(wheelSpeeds.leftMetersPerSecond);
    double rightFF = Constants.XRPDrivetrain.driveFFRight.calculate(wheelSpeeds.rightMetersPerSecond); 

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

    leftDist = 0; 
    rightDist = 0; 
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
    return (this.curLeftDist - this.leftDist) / Constants.robotPeriod; 
  }

  public double getRightSpeed() {
    return (this.curRightDist - this.rightDist) / Constants.robotPeriod;  
  }

  public double getLeftVolts() {
    return m_leftMotor.get() * RobotController.getInputVoltage(); 
  }

  public double getRightVolts() {
    return m_rightMotor.get() * RobotController.getInputVoltage(); 
  }

  public Field2d getField() {
    return this.m_field; 
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    this.odometry.update(getAngle(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance()); 

    this.m_field.setRobotPose(getPose());
    SmartDashboard.putData(this.m_field); 
    SmartDashboard.putNumber("gyro angle", getAngle().getDegrees()); 


    SmartDashboard.putNumber("Total Voltage", RobotController.getInputVoltage()); 

    SmartDashboard.putNumber("Left Volts", getLeftVolts()); 
    SmartDashboard.putNumber("Left Last Distance", leftDist); 
    SmartDashboard.putNumber("Left Distance", getLeftDistanceMeters()); 
    SmartDashboard.putNumber("Left Velo", getLeftSpeed()); 

    SmartDashboard.putNumber("Left Rate", m_leftEncoder.getRate());
    SmartDashboard.putNumber("Right Rate", m_rightEncoder.getRate()); 

    SmartDashboard.putNumber("Left Period", m_leftEncoder.getPeriod()); 
    SmartDashboard.putNumber("Right Period", m_rightEncoder.getPeriod()); 

    
    SmartDashboard.putNumber("Right Velo", getRightSpeed()); 
    
    super.periodic();

    // if (loopsCount >= maxLoopsCount) {
    //   loopsCount = 0; 
      leftDist = this.curLeftDist; 
      rightDist = this.curRightDist; 
    this.curLeftDist = getLeftDistanceMeters(); 
    this.curRightDist = getRightDistanceMeters(); 

    //   lastLeftSpeed = (this.curLeftDist - leftDist) / 0.02 / maxLoopsCount; 
    //   lastRightSpeed = (this.curRightDist - rightDist) / 0.02 / maxLoopsCount; 
    // } else loopsCount++; 


    
  }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        return Constants.XRPDrivetrain.kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed())); 
    }
}
