package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.Configs;

public class MAXSwerveModule extends SubsystemBase {

  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final CANcoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;

  // ✅ WPILib PID (replaces Spark turning PID)
  private final PIDController m_turningPID = new PIDController(1, 0.0, 0.0);

  private double m_CANCoderOffset = 0;

  public SwerveModuleState m_desiredState =
      new SwerveModuleState(0.0, new Rotation2d());

  public MAXSwerveModule(
      int drivingCANId,
      int turningCANId,
      int CANCoderId,
      double CANCoderOffset) {

    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = new CANcoder(CANCoderId);

    m_drivingClosedLoopController =
        m_drivingSpark.getClosedLoopController();

    // Apply configs
    m_drivingSpark.configure(
        Configs.MAXSwerveModule.drivingConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_turningSpark.configure(
        Configs.MAXSwerveModule.turningConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // CANCoder config
    m_turningEncoder.getConfigurator().apply(
        new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(1));

    m_CANCoderOffset = CANCoderOffset;

    // Reset drive encoder
    m_drivingEncoder.setPosition(0);

    // ✅ Enable continuous input (CRITICAL)
    m_turningPID.enableContinuousInput(0, 2 * Math.PI);

    // Optional tolerance
   
  }

  // =========================
  // STATE METHODS
  // =========================

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        new Rotation2d(getCANCoderAngle()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(getCANCoderAngle()));
  }

  // =========================
  // MAIN CONTROL
  // =========================

  public void setDesiredState(SwerveModuleState desiredState) {

    // Optimize to avoid spinning > 90°
    SwerveModuleState correctedDesiredState =
        SwerveModuleState.optimize(
            desiredState,
            new Rotation2d(getCANCoderAngle()));

    // Drive motor (unchanged)
    m_drivingClosedLoopController.setSetpoint(
        correctedDesiredState.speedMetersPerSecond,
        ControlType.kVelocity);

    // Turning control (NEW)
    double currentAngle = getCANCoderAngle();
    double targetAngle = correctedDesiredState.angle.getRadians();

    double output = m_turningPID.calculate(currentAngle, targetAngle);

   

// optional soft limit
output *= 0.5;

    // Optional deadband to reduce jitter
    if ( m_turningPID.atSetpoint()) {
      output = 0;
    }

    m_turningSpark.set(output);

    m_desiredState = desiredState;

    // Debugging
    SmartDashboard.putNumber("Target Angle", targetAngle);
    SmartDashboard.putNumber("Current Angle", currentAngle);
    SmartDashboard.putNumber("Turning Output", output);
  }

  // =========================
  // UTIL
  // =========================

  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  private double getCANCoderAngle() {
    return
        (m_turningEncoder.getAbsolutePosition().getValueAsDouble()
            - m_CANCoderOffset)*(2*Math.PI);
    
  }

  public void zeroModuleAngle() {
    m_turningPID.reset();
  }

  public void pointModuleAt(Rotation2d angle) {
    double output = m_turningPID.calculate(
        getCANCoderAngle(),
        angle.getRadians());

    m_turningSpark.set(output);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "CANCoder Angle",
        getCANCoderAngle());

    SmartDashboard.putNumber(
        "Raw CANCoder",
        m_turningEncoder.getAbsolutePosition().getValueAsDouble());
  }
}