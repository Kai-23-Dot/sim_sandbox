package frc.robot.subsystems.arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;
import frc.robot.util.sim.SimulatableMechanism;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;



public class ArmSubsystem extends SubsystemBase implements SimulatableMechanism {
    private final TalonFX armKraken = new TalonFX(ArmConfig.ARM_KRAKEN_ID, Constants.RIO_BUS);
    private final NeutralOut neutralOut = new NeutralOut();
    private final MotionMagicTorqueCurrentFOC magicRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
    private final MotorController armMotor = new PWMSparkMax(0);


    private CANcoder armEncoder;

    public ArmSubsystem() {
        armKraken.getConfigurator().apply(ArmConfig.talonFXConfiguration);
        armKraken.setControl(neutralOut);
        armEncoder = new CANcoder(ArmConfig.ARM_CANCODER_ID);
        armEncoder.getConfigurator().apply(ArmConfig.cancoderConfiguration);

        PhysicsSim.getInstance().addTalonFX(armKraken, armEncoder);

    }

    public void moveArmToPositionRotations(double desiredRotations) {
        double minRotations = 0.0;
        double maxRotations = 5.0;

        double clamped = Math.max(minRotations, Math.min(desiredRotations, maxRotations));

        armKraken.setControl(magicRequest.withPosition(clamped));
    }


    @Override
    public Angle getCurrentPosition() {
        return armKraken.getPosition().getValue();
    }

    @Override
    public Angle getTargetPosition() {
        return Units.Rotations.of(armKraken.getClosedLoopReference().getValue());
    }

    // ArmSubsystem.java
    public Command moveArm(double power) {
        return new RunCommand(() -> armKraken.setControl(new DutyCycleOut(power)), this);
    }

    public StatusCode moveTo(Angle setpoint) {
        return armKraken.setControl(magicRequest.withPosition(setpoint));
    }


}


