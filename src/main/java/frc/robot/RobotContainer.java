/*****************************************************************************
* Copyright (c) FIRST and other WPILib contributors.
* Open Source Software; you can modify and/or share it under the terms of
* the WPILib BSD license file in the root directory of this project.
*
* RobotContainer.java
*
* Configures the bindings, subsytems, and commands for the robot
*
*****************************************************************************/

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.LadderConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LadderJoystickCmd;
import frc.robot.commands.LadderMove;
import frc.robot.commands.LadderMoveAuto;
import frc.robot.commands.ResetLadderEncoder;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.LadderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/****************************************************************************************************
 * Class:
 *  RobotContainer
 * 
 * Purpose:
 *  This class contains all of the robot bindings, subsytems, and commands. Since Command-based
 *  is a "declarative" paradigm, very little robot logic should actually be handled in the
 *  {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure
 *  of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 * 
 ***************************************************************************************************/
public class RobotContainer {
  
  /* Private member variables */

  /* Subsystems */
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final LadderSubsystem m_ladderSubsystem = new LadderSubsystem();
  private final CameraSubsystem m_cameraSubsystem = new CameraSubsystem(); // Unused?

  /* Joysticks */
  private final Joystick m_driverJoystickOne = new Joystick(OIConstants.kDriverControllerOnePort);
  private final Joystick m_driverJoystickTwo = new Joystick(OIConstants.kDriverControllerTwoPort);

  /* Autonomous control */
  private final SendableChooser<Command> m_autoChooser;

  /* Public member variables */
  // None

  /****************************************************************************************************
  * Constructor:
  *  RobotContainer
  *
  * Purpose:
  *  Initialize robot fucntionality. Contains subsystems, OI devices, and commands.
  *
  * Parameters:
  *  None
  *
  * Return:
  *  None
  *
  ***************************************************************************************************/
  public RobotContainer() {

    // Initialize autonomous mode selection controls
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    //Creates all named commands for pathPlanner
    NamedCommands.registerCommand("LadderL1", new LadderMoveAuto(m_ladderSubsystem, LadderConstants.kLiftTroughSetPoint));
    NamedCommands.registerCommand("LadderRecieve", new LadderMoveAuto(m_ladderSubsystem, LadderConstants.kLiftRecieveSetPoint));
    NamedCommands.registerCommand("LadderL2", new LadderMoveAuto(m_ladderSubsystem, LadderConstants.kLiftLowSetPoint));
    NamedCommands.registerCommand("LadderL3", new LadderMoveAuto(m_ladderSubsystem, LadderConstants.kLiftMidSetPoint));
    NamedCommands.registerCommand("LadderL4", new LadderMoveAuto(m_ladderSubsystem, LadderConstants.kLiftHighSetPoint));

    //Initialize subsystem defaults
    m_swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
            m_swerveSubsystem,
            () -> -m_driverJoystickOne.getRawAxis(OIConstants.kDriverYAxis),
            () -> m_driverJoystickOne.getRawAxis(OIConstants.kDriverXAxis),
            () -> m_driverJoystickOne.getRawAxis(OIConstants.kDriverRotAxisXbox),
            //Side note: This could probably be improved to be a toggle to switch between field oriented and robot oriented.
            //This would make it so the user does not need to hold the button to drive robot oriented. See changes in SwerveJoystickCmd for an idea
            () -> m_driverJoystickTwo.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

    m_ladderSubsystem.setDefaultCommand(new LadderJoystickCmd(m_ladderSubsystem, () -> m_driverJoystickTwo.getRawAxis(OIConstants.kDriverYAxis) ));

    // Configure the joystick bindings
    configureBindings();

  } // RobotContainer()

  /****************************************************************************************************
  * Method:
  *  configureBindings
  *
  * Purpose:
  *  Configure non-default joystick controls to subsytem functionality.
  *
  * Parameters:
  *  None
  *
  * Return:
  *  None
  *
  ***************************************************************************************************/
  private void configureBindings() {
    // Configure joystick two to control ladder subsytem on button presses
    // Improvement suggestion: If multiple buttons are pressed at once, what sort of chaos happens?
    // Is it possible to not use these and to change LadderMove to take in m_driverJoystickTwo as a parameter?
    // If you can do that, inside LadderMove's execute function, check the current state of the buttons in priority order and update the setpoint accordingly, if no button is pressed fall back to the joystick input.
    // If that is possible, change setDefaultCommand above and then remove these
    new JoystickButton(m_driverJoystickTwo, OIConstants.kLiftHighButton).toggleOnTrue(new LadderMove(m_ladderSubsystem, LadderConstants.kLiftHighSetPoint));
    new JoystickButton(m_driverJoystickTwo, OIConstants.kLiftMidButton).toggleOnTrue(new LadderMove(m_ladderSubsystem, LadderConstants.kLiftMidSetPoint));
    new JoystickButton(m_driverJoystickTwo, OIConstants.kLiftLowButton).toggleOnTrue(new LadderMove(m_ladderSubsystem, LadderConstants.kLiftLowSetPoint));
    new JoystickButton(m_driverJoystickTwo, OIConstants.kliftTroughButton).toggleOnTrue(new LadderMove(m_ladderSubsystem, LadderConstants.kLiftTroughSetPoint));
    new JoystickButton(m_driverJoystickTwo, OIConstants.kLiftRecieveButton).toggleOnTrue(new LadderMove(m_ladderSubsystem, LadderConstants.kLiftRecieveSetPoint));

    new JoystickButton(m_driverJoystickTwo, OIConstants.kLiftResetEncoderButton).whileTrue(new ResetLadderEncoder(m_ladderSubsystem));


    // Unused
    // /*
    // new JoystickButton(driverJoystickOne, OIConstants.kLiftHighButton).whileTrue(new LadderHigh(ladderSubsystem, LadderConstants.kLiftHighSetPoint));
    // new JoystickButton(driverJoystickOne, OIConstants.kLiftMidButton).whileTrue(new LadderMid(ladderSubsystem, LadderConstants.kLiftMidSetPoint));
    // new JoystickButton(driverJoystickOne, OIConstants.kLiftLowButton).whileTrue(new LadderLow(ladderSubsystem, LadderConstants.kLiftLowSetPoint));
    // new JoystickButton(driverJoystickOne, OIConstants.kliftTroughButton).whileTrue(new LadderTrough(ladderSubsystem, LadderConstants.kLiftTroughSetPoint));
    // new JoystickButton(driverJoystickOne, OIConstants.kLiftRecieveButton).whileTrue(new LadderRecieve(ladderSubsystem, LadderConstants.kLiftRecieveSetPoint));
    // */
    // //these two button move the ladder without a setPoint
    // //new JoystickButton(driverJoystickTwo, OIConstants.kliftSpeedUpButton).whileTrue(new LadderShift(ladderSubsystem, LadderConstants.kLiftSpeedUp));
    // //new JoystickButton(driverJoystickTwo, OIConstants.kliftSpeedDownButton).whileTrue(new LadderShift(ladderSubsystem, LadderConstants.kliftSpeedDown));
  } // configureBindings()

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  /****************************************************************************************************
  * Method:
  *  getAutonomousCommand
  *
  * Purpose:
  *  Pass the autonomous command to the main {@link Robot} class.
  *
  * Parameters:
  *  None
  *
  * Return:
  *  The command to run in autonomous
  *
  ***************************************************************************************************/
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
  //  return Autos.exampleAuto(m_exampleSubsystem);
  return m_autoChooser.getSelected();
  } // getAutonomousCommand()
}