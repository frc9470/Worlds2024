package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends VerticalArm {
    private final CANSparkMax roller;

    public Intake() {
        super(new CANSparkMax(INTAKE_ARM, CANSparkLowLevel.MotorType.kBrushless), ARM_PID, ARM_FF, 0, 1 / 48.0, ARM_ABSOLUTE_OFFSET);
        roller = new CANSparkMax(INTAKE_ROLLER, CANSparkLowLevel.MotorType.kBrushless);

        roller.restoreFactoryDefaults();
        roller.setIdleMode(CANSparkMax.IdleMode.kCoast);

        arm.setInverted(ARM_INVERTED);
        roller.setInverted(ROLLER_INVERTED);

        arm.setSmartCurrentLimit(40);
        roller.setSmartCurrentLimit(20);

        initTelemetry();
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Intake Position:", getArmPos());

    }

    public void initTelemetry() {
        SmartDashboard.putNumber("Arm position:", getArmPos());
        SmartDashboard.putNumber("Arm position (rad):", getArmPosRad());
        SmartDashboard.putNumber("Intake Position:", getArmPos());
    }

    public Command armToGround() {
        return armToPos(() -> ARM_GROUND_POSITION);
    }

    public Command armToTransfer() {
        return armToPos(() -> ARM_STOW_POSITION);
    }

    public Command setIntake(double speed) {
        return this.runEnd(() -> roller.set(speed), () -> roller.set(0));
    }

    public Command rollerIn() {return setIntake(ROLLER_INTAKE_SPEED);}
    public Command rollerOut() {return setIntake(ROLLER_OUTTAKE_SPEED);}
}
