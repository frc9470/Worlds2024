package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FFConstants;
import frc.robot.PIDConstants;

import java.util.function.Supplier;

public abstract class VerticalArm extends SubsystemBase {
    protected final CANSparkMax arm;
    protected final DutyCycleEncoder throughBore;
    protected final RelativeEncoder armEncoder;
    protected final PIDController armPID;
    protected final ArmFeedforward armFF;

    public VerticalArm(CANSparkMax arm, PIDConstants pidConstants, FFConstants armFF) {
        this.arm = arm;
        arm.restoreFactoryDefaults();
        arm.setIdleMode(CANSparkMax.IdleMode.kBrake);
        arm.setInverted(false);
        arm.setSmartCurrentLimit(40);

        this.armPID = pidConstants.getController();
        this.armFF = armFF.getArmFF();

        throughBore = new DutyCycleEncoder(0);
        armEncoder = arm.getEncoder();
    }

    public void resetEncoder() {
        armEncoder.setPosition(throughBore.getAbsolutePosition());
    }

    public void setArmSetpoint(double pos) {
        armPID.setSetpoint(pos);
    }

    public double getArmPosRad() {
        return 2 * Math.PI * armEncoder.getPosition();
    }

    public double getArmPos() {
        return armEncoder.getPosition();
    }

    @Override
    public void periodic() {
        //move the arm to the setpoint every 20 ms
        double output = armPID.calculate(armEncoder.getPosition()) + armFF.calculate(getArmPosRad(), 0);
        arm.set(output);
        SmartDashboard.putNumber("Error", getArmPos() - armPID.getSetpoint());
        SmartDashboard.putNumber("Rotations", armPID.getSetpoint());
        SmartDashboard.putNumber("Output", output);
        SmartDashboard.putNumber("Count", SmartDashboard.getNumber("Count", 0) + 1);
    }

    /*********************************************************************************
     ********************************* COMMANDS **************************************
     *********************************************************************************/

    public Command armToPos(Supplier<Double> rotationSupplier) {
        return new Command() {
            @Override
            public void initialize() {
                super.initialize();
                setArmSetpoint(rotationSupplier.get());
            }

            @Override
            public boolean isFinished() {
                return armPID.atSetpoint();
            }
        };
    }
}
