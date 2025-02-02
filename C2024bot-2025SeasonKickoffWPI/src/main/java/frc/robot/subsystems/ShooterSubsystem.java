// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimiter;
import frc.robot.Constants.EnableCurrentLimiter;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.GPMConstants.Shooter;
import frc.robot.Constants.GPMConstants.Shooter.ShooterMotorConstantsEnum;
import frc.robot.Constants.GPMConstants.Shooter.ShooterPIDConstants;

public class ShooterSubsystem extends SubsystemBase {

  private InterpolatingDoubleTreeMap SHOOTER_POWER = new InterpolatingDoubleTreeMap();

  // NEO motors connected to Spark Max
  private SparkMax shooterMotorLeft;
  private SparkMax shooterMotorRight;
  private SparkMax shooterMotorLeader;


  // Necessary for hardware PID with Spark Max
  private SparkClosedLoopController shooterPIDControllerLeft;
  private SparkClosedLoopController shooterPIDControllerRight;

  // Built-in NEO encoders
  // Will be used with Velocity PID
  private RelativeEncoder shooterEncoderLeft;
  private RelativeEncoder shooterEncoderRight;


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    // Check if need to initialize shooter
    if (! EnabledSubsystems.shooter) { return; }

    shooterMotorLeft = new SparkMax(ShooterMotorConstantsEnum.LEFTMOTOR.getShooterMotorID(), MotorType.kBrushless);
    shooterMotorRight = new SparkMax(ShooterMotorConstantsEnum.RIGHTMOTOR.getShooterMotorID(), MotorType.kBrushless);

    shooterPIDControllerLeft = shooterMotorLeft.getClosedLoopController();
    shooterPIDControllerRight = shooterMotorRight.getClosedLoopController();

    shooterEncoderLeft = shooterMotorLeft.getEncoder();
    shooterEncoderRight = shooterMotorRight.getEncoder();

    // Main Motor; should not follow the other motor
    configureshooterMotors(shooterMotorLeft, shooterEncoderLeft, shooterPIDControllerLeft, ShooterMotorConstantsEnum.LEFTMOTOR, null);
    // Follower Motor
    configureshooterMotors(shooterMotorRight, shooterEncoderRight, shooterPIDControllerRight, ShooterMotorConstantsEnum.RIGHTMOTOR,
        shooterMotorLeft);
    
    //shooterMotorLeft.setIdleMode(IdleMode.kCoast);

    setShooterPower();
    
    System.out.println("*** Shooter initialized");

  }

  /**
   * Configure Shooter motors with a main and a follower
   * 
   * @param motor         - motor object
   * @param p             - PID controller object
   * @param c             - motor constants
   * @param motorToFollow - motor to follow if this is a follower
   */
  private void configureshooterMotors(SparkMax motor, RelativeEncoder encoder, SparkClosedLoopController p, ShooterMotorConstantsEnum c,
      SparkMax motorToFollow) {

    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

    //motor.restoreFactoryDefaults();
    motor.clearFaults();
    sparkMaxConfig.inverted(c.getShooterMotorInverted());

    sparkMaxConfig.idleMode(IdleMode.kBrake);

    EncoderConfig encoderConfig = new EncoderConfig();
    encoderConfig.positionConversionFactor(Shooter.POSITION_CONVERSION_FACTOR);
    encoderConfig.velocityConversionFactor(Shooter.VELOCITY_CONVERSION_FACTOR);
    sparkMaxConfig.apply(encoderConfig);

    motor.setCANTimeout(0);

    sparkMaxConfig.voltageCompensation(Shooter.nominalVoltage);

    if (EnableCurrentLimiter.shooter) {
      sparkMaxConfig.smartCurrentLimit(CurrentLimiter.shooter);
    }
    
    sparkMaxConfig.openLoopRampRate(Shooter.rampRate);
    sparkMaxConfig.closedLoopRampRate(Shooter.rampRate);

    SignalsConfig signalsConfig = new SignalsConfig();

    // sets which motor is the leader and follower; set follower inversion if needed
    if (c.getShooterMotorFollower()) {
      sparkMaxConfig.follow(motorToFollow,c.getShooterMotorInverted());

      //motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
      //motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 250);
      //motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 250);

      // kstatus0
      signalsConfig.faultsPeriodMs(100);
      signalsConfig.appliedOutputPeriodMs(100);
      signalsConfig.outputCurrentPeriodMs(100);
      
      // kstatus1
      signalsConfig.motorTemperaturePeriodMs(250);
      signalsConfig.primaryEncoderVelocityPeriodMs(250);

      //kstatus2
      signalsConfig.primaryEncoderPositionPeriodMs(250);

    } else {
      shooterMotorLeader = motor;

      //motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 25);
      //motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 50);
      //motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);

      // kstatus1
      signalsConfig.motorTemperaturePeriodMs(50);
      signalsConfig.primaryEncoderVelocityPeriodMs(50);

      // kstatus2
      signalsConfig.primaryEncoderPositionPeriodMs(50);

    }

    // apply signals
    sparkMaxConfig.apply(signalsConfig);

    ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();

    closedLoopConfig.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    closedLoopConfig.p(ShooterPIDConstants.kP);
    closedLoopConfig.i(ShooterPIDConstants.kI);
    closedLoopConfig.d(ShooterPIDConstants.kD);
    closedLoopConfig.iZone(ShooterPIDConstants.Izone);

    closedLoopConfig.outputRange(-ShooterPIDConstants.kMaxOutput, ShooterPIDConstants.kMaxOutput);

    sparkMaxConfig.apply(closedLoopConfig);

    motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // // PID Controller setup
    // p.setPositionPIDWrappingEnabled(false);
    // p.setP(ShooterPIDConstants.kP);
    // p.setI(ShooterPIDConstants.kI);
    // p.setD(ShooterPIDConstants.kD);
    // p.setIZone(ShooterPIDConstants.Izone);
    // p.setFF(ShooterPIDConstants.kF);
    // // kMaxOutput = 1 ; range is -1, 1
    // p.setOutputRange(-ShooterPIDConstants.kMaxOutput, ShooterPIDConstants.kMaxOutput);

    // kMaxOutput = 1 ; range is -1, 1
    // shooterPIDControllerB.setOutputRange(-Constants.GPMConstants.ShooterPIDConstants.kMaxOutput,
    // Constants.GPMConstants.ShooterPIDConstants.kMaxOu
  }

  /**
   * Run shooter with velocity using PID
   * @param speed
   */
  public void runShooterWithVelocity(double speed) {
    shooterMotorLeader.getClosedLoopController().setReference((speed), ControlType.kVelocity);
  }

  
  /* 
  public double convertShooterPowerIntoShooterSpeed(double power) {
    return power;
  }
  */

  /* 
  public double convertShooterSpeedIntoShooterPower(double speed) {
    return speed;
  }
  */

  /**
   * Run shooter with NON-PID power -1..1
   * @param power
   */
  public void runShooterWithPower(double power) {
    shooterMotorLeader.set(power);
  }

  /**
   * Run shooter with PID power -1..1; converts power to voltage
   * @param power
   */
  public void runShooterWithPowerPID(double power) {
    runShooterWithVoltagePID(MathUtil.clamp (power * Shooter.nominalVoltage, -Shooter.nominalVoltage, Shooter.nominalVoltage));
  }

  /**
   * Run shooter with PID voltage; clamp voltage to nominal range
   * @param voltage
   */
  public void runShooterWithVoltagePID(double voltage) {
    shooterMotorLeader.getClosedLoopController().setReference(MathUtil.clamp (voltage, -Shooter.nominalVoltage, Shooter.nominalVoltage), ControlType.kVoltage);
  }

  public void stopShooter() {
    //shooterMotorLeader.getPIDController().setReference((0), ControlType.kVelocity);
    shooterMotorLeader.set(0);
  }

  // ===============================
  // ===== Shooter telemetry methods
  // ===============================

  public double getLeftShooterMotorVelocity() {
    return shooterEncoderLeft.getVelocity();
  }

  public double getRightShooterMotorVelocity() {
    return shooterEncoderRight.getVelocity();
  }

    public double getLeftShooterMotorEncoder() {
    return shooterEncoderLeft.getPosition();
  }

  public double getRightShooterMotorEncoder() {
    return shooterEncoderRight.getPosition();
  }

  public void setShooterPower() {
    SHOOTER_POWER.put(0.0, 0.0); //TODO: need to calibrate
    SHOOTER_POWER.put(6.0, 0.8); 
  }

  public double getLeftShooterMotorTemp() {
    return shooterMotorLeft.getMotorTemperature();
  }

  public double getRightShooterMotorTemp() {
    return shooterMotorRight.getMotorTemperature();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
