// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakePivotSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakePivotCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakePivotSubsystem m_subsystem;
  private final XboxController m_Controller;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakePivotCommand(IntakePivotSubsystem subsystem, XboxController controller) {
    m_subsystem = subsystem;
    m_Controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((m_Controller.getAButton()) ){
        m_subsystem.intake(-0.7);
          m_subsystem.setAngle(76);
        
    // } else if((m_Controller.getAButton()) && (m_subsystem.getSensor())) {
    //     m_subsystem.intake(-0.5);
    //     m_subsystem.setAngle(5.5);
    }else{
        m_subsystem.setAngle(6);
        m_subsystem.intake(0.0);
    }

    if(m_Controller.getXButton()){
      m_subsystem.intake(0.7);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}
