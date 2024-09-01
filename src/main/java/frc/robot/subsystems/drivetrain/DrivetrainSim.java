package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class DrivetrainSim extends BaseDrivetrain {


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


  // simulation!!
  private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder); 
  private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder); 

  private SimDeviceSim m_gyroSim = new SimDeviceSim("Gyro:RomiGyro");

private DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
  // Create a linear system from our identification gains.
//   LinearSystemId.createDrivetrainVelocitySystem(DCMotor.getRomiBuiltIn(1), 0.215, (Constants.Drivetrain.kWheelDiameterMeters) / 2, 0.141/2, 0.5*0.215*Math.hypot(0.163, 0.149), 120),
  DCMotor.getRomiBuiltIn(1),       // 1 ROMI motors on each side of the drivetrain.
  1,                               // 120:1 gearing reduction.
  0.5*0.215*Math.pow(Math.hypot(0.163, 0.149), 2), // moment of inertia
  0.215, 
  (Constants.RomiDrivetrain.kWheelDiameterMeters) / 2, // The robot uses 35mm radius wheels.
  0.141,                   // The track width is 141 mm.
null);

  /** Creates a new RomiDrivetrain. */
  public DrivetrainSim() {
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

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    m_driveSim.setInputs(getLeftVolts(), getRightVolts());
    m_driveSim.update(0.02);
    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    m_gyroSim.getDouble("angle_z").set(m_driveSim.getHeading().getDegrees());
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
