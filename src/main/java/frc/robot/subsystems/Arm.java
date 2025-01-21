package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj.Encoder;


import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private final WPI_TalonSRX m_arm = new WPI_TalonSRX(Constants.kArmMotorPort);

  private final Encoder m_Encoder =
  new Encoder(
    Constants.kEncoderAChannel,
    Constants.kEncoderBChannel,
    false
    );

  public void resetEncoder() {
    m_Encoder.reset();
  }

  public int getEncoder() {
    return m_Encoder.get();
  }

  public double getArmPosition() {
    return m_Encoder.getDistance();
  }
  
  public void SetArm(double degrees) {
    m_arm.set(degrees);
  }

}
