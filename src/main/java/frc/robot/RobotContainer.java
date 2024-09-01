// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveRomi;
import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainSim;
import frc.robot.subsystems.drivetrain.XRPDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final BaseDrivetrain m_romiDrivetrain = new XRPDrivetrain();

  private final CommandXboxController controller = new CommandXboxController(0); 

  private SendableChooser<Command> auto; 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    this.auto = AutoBuilder.buildAutoChooser(); 

    SmartDashboard.putData(auto);

    m_romiDrivetrain.setDefaultCommand(new DriveRomi(m_romiDrivetrain, () -> controller.getLeftY(), () -> controller.getRightX()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // controller.a().whileTrue(Commands.run(() -> m_romiDrivetrain.driveVoltage(1.3, 1.70), m_romiDrivetrain));
    controller.a().whileTrue(m_romiDrivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    controller.b().whileTrue(m_romiDrivetrain.sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
    controller.x().whileTrue(m_romiDrivetrain.sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
    controller.y().whileTrue(m_romiDrivetrain.sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse)); 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return auto.getSelected();
  }
}
