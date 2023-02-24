// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  // TalonSRX intakeMotor;
  VictorSP intakeMotor;
  CANSparkMax intakeLiftMotor;

  RelativeEncoder intakeLiftEncoder;
  SparkMaxPIDController intakeLiftPID; 

  private LinearSystem<N2, N1, N1> intakeArm;
  private KalmanFilter<N2, N1, N1> armObserver;
  private LinearQuadraticRegulator<N2, N1, N1> armController;

  private LinearSystemLoop<N2, N1,N1> armLoop;

  private double JKgSquaredMeters = 0.29;
  private double intakeGearing = 45.0;

  public Intake() {
    intakeMotor = new VictorSP(Constants.IntakeConstants.kIntakeMotor); // PWM channels
    intakeLiftMotor = new CANSparkMax(Constants.IntakeConstants.kLiftMotor, MotorType.kBrushless);
    intakeLiftMotor.setIdleMode(IdleMode.kBrake);

    intakeLiftEncoder = intakeLiftMotor.getEncoder();
    intakeLiftPID = intakeLiftMotor.getPIDController();
  
  }

  public void runIntakeForward(){

  }
  public void runIntakeReverse(){

  }

  public void setIntake(){
    intakeMotor.set(0.2);
  }

  public void setReverse(){
    intakeMotor.set(-0.2);
  }

  public void setIntakePivot(double degrees){

    intakeArm = LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(1), JKgSquaredMeters, intakeGearing);

    armObserver = new KalmanFilter<>(Nat.N2(),Nat.N1() ,intakeArm, VecBuilder.fill(0.2, 0.2), VecBuilder.fill(0.01), 0.3);
    
    armController =
    new LinearQuadraticRegulator<>(
        intakeArm,
        VecBuilder.fill(0.1,10.0), // qelms. Velocity error tolerance, in radians per second. Decrease
        // this to more heavily penalize state excursion, or make the controller behave more
        // aggressively.
        VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
        // heavily penalize control effort, or make the controller less aggressive. 12 is a good
        // starting point because that is the (approximate) maximum voltage of a battery.
        0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
// lower if using notifiers.

    // The state-space loop combines a controller, observer, feedforward and plant for easy control.
    armLoop = new LinearSystemLoop<>(intakeArm, armController, armObserver, 12.0, 0.020);

    

    

  }

  public void stopIntake(){
    intakeMotor.stopMotor();
  }

  public void liftIntake(){
    intakeLiftMotor.set(0.1);

  }
  public void lowerIntake(){
    intakeLiftMotor.set(-0.1);
  }

  public void stopIntakeLift(){
    intakeLiftMotor.stopMotor();
  }

  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
