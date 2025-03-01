package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorSubsystem extends SubsystemBase {
  
  private static double kDt = 0.02; //some sort of timing thing, don't touch probably

  private final SparkMax elevatorOneSparkMax = new SparkMax(14, MotorType.kBrushless);
  SparkMaxConfig elevatorOneConfig = new SparkMaxConfig();

  private final SparkMax elevatorTwoSparkMax = new SparkMax(15, MotorType.kBrushless);
  SparkMaxConfig elevatorTwoConfig = new SparkMaxConfig();

  private final RelativeEncoder mElevatorEncoder;
  private final SparkClosedLoopController mElevatorPID;

  private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(600, 600));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  public ElevatorSubsystem() {
    //Configure elevator motor 1 
    elevatorOneConfig.idleMode(IdleMode.kBrake);
    elevatorOneConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.3, 0, 0); 
    elevatorOneConfig.encoder.positionConversionFactor(1);//(360.0/(60.0));
    elevatorOneConfig.encoder.velocityConversionFactor(1); //(360.0/(60.0*10));
    elevatorOneSparkMax.configure(elevatorOneConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

    //Set initial encoder position to 0
    mElevatorEncoder = elevatorOneSparkMax.getEncoder();
    mElevatorEncoder.setPosition(0);
    mElevatorPID = elevatorOneSparkMax.getClosedLoopController();

    // Configure motor 2 to follow motor 1
    elevatorTwoSparkMax.configure(elevatorOneConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Calculate the set point for the elevator
    m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);

    // New posistion to the PID controller
    mElevatorPID.setReference(m_setpoint.position, com.revrobotics.spark.SparkBase.ControlType.kPosition);
  }

    // set target position for the elevator
    public void setPosistion(double posistion) {
        m_goal = new TrapezoidProfile.State(posistion, 0);
    }

    public void doNothing(){}
    
    public Command elevatorUp(int level){
      // move the elevator up
      if (level == 1){
        return this.startEnd(() -> setPosistion(1), () -> doNothing()).until(() -> moveInPosistion());
      }
      else if (level == 2){
        return this.startEnd(() -> setPosistion(2), () -> doNothing()).until(() -> moveInPosistion());
      }
      else if (level == 3){
        return this.startEnd(() -> setPosistion(3), () -> doNothing()).until(() -> moveInPosistion());
      }
      else if (level == 4){
        return this.startEnd(() -> setPosistion(4), () -> doNothing()).until(() -> moveInPosistion());
      }
      else{
        return this.startEnd(() -> setPosistion(0), () -> doNothing()).until(() -> moveInPosistion());
      }
    }

    public Command elevatorDown(){
      // move the elevator down
      return this.startEnd(() -> setPosistion(0), () -> doNothing()).until(() -> moveInPosistion());
    }

    public boolean moveInPosistion() {
      return Math.abs(m_setpoint.position - m_goal.position) < 5;
    }
}
