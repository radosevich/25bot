package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



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
  
  public void MoveArm(double speed) {
    m_arm.set(speed);
  }

}
