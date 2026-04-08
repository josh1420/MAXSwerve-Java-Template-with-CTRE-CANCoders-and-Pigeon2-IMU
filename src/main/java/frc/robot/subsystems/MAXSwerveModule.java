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

  /** PID controller for rotating the swerve module wheel to the desired angle.
   *  I recommend starting with these values as a value greater than 1 will cause the modules to recoil to position */
  private final PIDController m_turningPID = new PIDController(0.5,0,0.0);

  /** The offset (in rotations) applied to the CANCoder reading to align the zero position of the wheel. */
  private double m_CANCoderOffset;

  /** Stores the last commanded state of this module. Initialized to zero speed and zero angle on startup. */
  public SwerveModuleState m_desiredState =
      new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule with the given motor and encoder CAN IDs and CANCoder offset.
   *
   * @param drivingCANId  CAN ID of the drive SparkMax motor controller.
   * @param turningCANId  CAN ID of the turning SparkMax motor controller.
   * @param CANCoderId    CAN ID of the CANCoder absolute encoder used for wheel angle.
   * @param CANCoderOffset Offset in rotations to zero the CANCoder to the wheel's forward position.
   */
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

    m_drivingSpark.configure(
        Configs.MAXSwerveModule.drivingConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_turningSpark.configure(
        Configs.MAXSwerveModule.turningConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Configures the CANCoder to report absolute position as a value between 0 and 1 (full rotation).
    m_turningEncoder.getConfigurator().apply(
        new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(1));

    m_CANCoderOffset = CANCoderOffset;

    m_drivingEncoder.setPosition(0);

    // Enables continuous input so the PID takes the shortest rotational path to the target angle,
    // avoiding unnecessary full 360° rotations.
    m_turningPID.enableContinuousInput(0, 2 * Math.PI);
    m_turningPID.reset();
  }

  /**
   * Returns the current state of this swerve module.
   *
   * @return A {@link SwerveModuleState} containing the current wheel velocity (m/s) and heading angle.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        new Rotation2d(getCANCoderAngle()));
  }

  /**
   * Returns the current position of this swerve module.
   *
   * @return A {@link SwerveModulePosition} containing the current distance traveled (meters) and current heading angle of the wheel.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(getCANCoderAngle()));
  }

  /**
   * Tells the swerve module how fast to go and where to point the wheel.
   * @param desiredState The target {@link SwerveModuleState} containing desired speed (m/s) and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    desiredState.optimize(new Rotation2d(getCANCoderAngle())); //Optimizes the module so it spins to the shortest path of the setpoint

    m_drivingClosedLoopController.setSetpoint(
        desiredState.speedMetersPerSecond,
        ControlType.kVelocity); //tells the module how fast to spin the wheel via Velocity Control

    double output = m_turningPID.calculate(getCANCoderAngle(), desiredState.angle.getRadians());
    //tells the module where to point the wheel
    //output*=0.5;
    m_turningSpark.set(output);
    if (m_turningPID.atSetpoint()){
      output=0;
    }

    m_desiredState = desiredState;
  }

  /**
   * Resets the drive encoder position to zero.
   * Call this to clear accumulated distance, typically at the start of a match or autonomous routine.
   */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current wheel heading in radians, adjusted by the CANCoder offset.
   * The raw CANCoder value (0–1 rotations) is offset and converted to radians (0–2π).
   *
   * @return The current wheel angle in radians.
   */
  private double getCANCoderAngle() {
    return
        (m_turningEncoder.getAbsolutePosition().getValueAsDouble()
            - m_CANCoderOffset) * (2 * Math.PI);
  }

  /**
   * Resets the turning PID controller's accumulated state.
   * Useful when re-enabling the robot or recovering from a disabled state to prevent integral windup.
   */
  public void zeroModuleAngle() {
    m_turningPID.reset();
  }

  /**
   * Rotates the module wheel to point at a specific angle using the turning PID controller.
   *
   * @param angle The target {@link Rotation2d} angle to point the wheel toward.
   */
  public void pointModuleAt(Rotation2d angle) {
    double output = m_turningPID.calculate(
        getCANCoderAngle(),
        angle.getRadians());

    m_turningSpark.set(output);
  }

  @Override
  public void periodic() {
  }
}