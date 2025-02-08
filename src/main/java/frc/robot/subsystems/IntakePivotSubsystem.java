// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivotSubsystem extends SubsystemBase {
  
  private final DigitalInput noteSensor = new DigitalInput(0);

  private static double kDt = 0.02; //some sort of timing thing, don't touch probably

  /** Creates a new ExampleSubsystem. */
 private final CANSparkMax mIntakePivot = new CANSparkMax(13, MotorType.kBrushless);
  private final CANSparkMax mIntakeDrive = new CANSparkMax(14, MotorType.kBrushless);

  private final RelativeEncoder mIntakeEncoder;
    private final SparkPIDController mPivotPID;
private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(600, 600));
private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  public IntakePivotSubsystem() {
        // Configure drive motor controller parameters
        mPivotPID = mIntakePivot.getPIDController();
        mPivotPID.setP(0.3);
        mPivotPID.setI(0);
        mPivotPID.setD(0);
      
        mIntakeEncoder = mIntakePivot.getEncoder();
        mIntakeEncoder.setPositionConversionFactor(360.0/(60.0));
        mIntakeEncoder.setPosition(0);

        mIntakePivot.setIdleMode(IdleMode.kBrake);
        mIntakePivot.setSmartCurrentLimit(30);
        mIntakePivot.setClosedLoopRampRate(0);
        mIntakePivot.setOpenLoopRampRate(.1);
        mIntakePivot.burnFlash();

        mIntakeDrive.setIdleMode(IdleMode.kBrake);
        mIntakeDrive.setSmartCurrentLimit(30);
        mIntakeDrive.setClosedLoopRampRate(0);
        mIntakeDrive.setOpenLoopRampRate(.1);
        mIntakeDrive.burnFlash();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
    mPivotPID.setReference(m_setpoint.position, com.revrobotics.CANSparkBase.ControlType.kPosition);
  }

      // set angle of swerve drive
    public void setAngle(double angle) {
        m_goal = new TrapezoidProfile.State(angle, 0);
    }

    public void doNothing(){

    }

    public void intake(double speed){
      mIntakeDrive.set(speed);
    }

    public boolean getSensor(){
      return(noteSensor.get());
    }
    

    public Command intakeUp(){
      return this.startEnd(() -> setAngle(5.5), () -> doNothing()).until(() -> pivotInPosition());
    }

    public Command intakeDown(){
      return this.startEnd(() -> setAngle(75.8), () -> doNothing()).until(() -> pivotInPosition());
    }

    public boolean pivotInPosition() {
      return Math.abs(m_setpoint.position - m_goal.position) < 5;
    }
    
    public Command intakeNote(){
      return this.startEnd(() -> intake(-0.6), () -> doNothing()).until(() -> getSensor());
    }

    public Command stopthestupidinatkething(){
      return this.runOnce (() -> intake(0.0));
    }

     public Command putThatThingBackWhereItCameFromOrSoHelpMe(){
      return this.runOnce (() -> intake(0.7));
    }   
}
