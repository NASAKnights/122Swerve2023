// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  // TalonSRX intakeMotor;
  VictorSP intakeMotor;
  CANSparkMax intakeLiftMotor;

  RelativeEncoder intakeLiftEncoder;
  SparkMaxPIDController intakeLiftPID; 

  private DigitalInput limitSwitch;

  private LinearSystem<N2, N1, N1> intakeArm;
  private KalmanFilter<N2, N1, N1> armObserver;
  private LinearQuadraticRegulator<N2, N1, N1> armController;

  private LinearSystemLoop<N2, N1,N1> armLoop;

  private double JKgSquaredMeters = 0.29;
  private double intakeGearing = 45.0;
  // private double stowedPosition = 3.14;
  private double stowedPosition = 0.0;
  private double extendedPosition = 3.14;
  private double acceptableError = 0.5;

  public Intake() {
    intakeMotor = new VictorSP(Constants.IntakeConstants.kIntakeMotor); // PWM channels
    intakeLiftMotor = new CANSparkMax(Constants.IntakeConstants.kLiftMotor, MotorType.kBrushless);
    intakeLiftMotor.setIdleMode(IdleMode.kBrake);
    intakeLiftMotor.setInverted(true);

    intakeLiftEncoder = intakeLiftMotor.getEncoder();
    intakeLiftPID = intakeLiftMotor.getPIDController();
    limitSwitch = new DigitalInput(0);

    intakeLiftEncoder.setPositionConversionFactor((1.0/45.0) * 2*Math.PI); // radian
    intakeLiftEncoder.setVelocityConversionFactor((1.0/45.0) * 2*Math.PI);
    
    resetPivotEncoder();

    intakeArm = LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(1), JKgSquaredMeters, intakeGearing);

    armObserver = new KalmanFilter<>(Nat.N2(),Nat.N1() ,intakeArm, VecBuilder.fill(0.2, 0.2), VecBuilder.fill(0.01), 0.02);
    
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

  public void runIntakeForward(){
    // takes in cone, outputs cube
    intakeMotor.set(0.6);

  }
  public void runIntakeReverse(){
    // outputs cone, takes in cube
    intakeMotor.set(-0.6);
  }

  public void setIntake(){
    intakeMotor.set(0.8);
  }

  public void handOffCube() {
    intakeMotor.set(0.1);
  }

  public void setReverse(){
    intakeMotor.set(-0.8);
  }

  public boolean isIntakeInside(){
    return !limitSwitch.get();
  }

  public void resetPivotEncoder(){
    intakeLiftEncoder.setPosition(0.0);
  }

  public double checkAngle(){
    return intakeLiftEncoder.getPosition();
  }

  public void setIntakePivot(double angle){
    if (angle > Math.PI * 1.1){
      return;
    }
    double speeds = (intakeLiftEncoder.getPosition() - angle);
    // double speeds = Math.min(0.3, intakeLiftEncoder.getPosition() - angle);
    armLoop.setNextR(VecBuilder.fill(angle, speeds));
    armLoop.correct(VecBuilder.fill(intakeLiftEncoder.getVelocity()));
    armLoop.predict(0.020);
    double nextVoltage = armLoop.getU(0);
    intakeLiftMotor.setVoltage(nextVoltage);
  }

  public void stowIntake(){
    if(Math.abs(intakeLiftEncoder.getPosition() - stowedPosition) > acceptableError)
    {
      // setIntakePivot(stowedPosition);
      // setReverse();
      lowerIntake();
    }
  }

  public void stopIntake(){
    intakeMotor.stopMotor();
  }

  public void liftIntake(){
    intakeLiftMotor.set(0.3);

  }
  public void lowerIntake(){
    intakeLiftMotor.set(-0.15);
  }


  public void stopIntakeLift(){
    intakeLiftMotor.stopMotor();
  }

  public void updateBoard(){
    SmartDashboard.putNumber("Intake Position", intakeLiftEncoder.getPosition());
    SmartDashboard.putBoolean("Intake Limit", isIntakeInside());
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (isIntakeInside()){
      resetPivotEncoder();
    }
  }
}