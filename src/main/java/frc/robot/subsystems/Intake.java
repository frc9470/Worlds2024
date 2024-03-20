package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.Supplier;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    private final CANSparkMax arm;
    private final CANSparkMax roller;
    private final DutyCycleEncoder throughBore;
    private final RelativeEncoder armEncoder;
    private final PIDController armPID;
    private final ArmFeedforward armFF;
    public Intake() {
        arm = new CANSparkMax(INTAKE_ARM, CANSparkLowLevel.MotorType.kBrushless);
        roller = new CANSparkMax(INTAKE_ROLLER, CANSparkLowLevel.MotorType.kBrushless);

        arm.restoreFactoryDefaults();
        roller.restoreFactoryDefaults();

        arm.setIdleMode(CANSparkMax.IdleMode.kBrake);
        roller.setIdleMode(CANSparkMax.IdleMode.kCoast);

        arm.setInverted(ARM_INVERTED);
        roller.setInverted(ROLLER_INVERTED);

        arm.setSmartCurrentLimit(40);
        roller.setSmartCurrentLimit(20);

        throughBore = new DutyCycleEncoder(0);

        armEncoder = arm.getEncoder();
        armEncoder.setPositionConversionFactor(1.0 / 48.0);

        armPID = new PIDController(PID_CONSTANTS.kP, PID_CONSTANTS.kI, PID_CONSTANTS.kD, PID_CONSTANTS.iZone);
        armPID.setTolerance(ARM_TOLERANCE);
        armFF = new ArmFeedforward(ARM_KS, ARM_G, ARM_V, ARM_A);


        resetEncoder();
        initTelemetry();

        setArmSetpoint(armEncoder.getPosition());
    }


    public void resetEncoder() {
        armEncoder.setPosition(throughBore.getAbsolutePosition());
    }
    public void initTelemetry() {
        //do logs later tycho

    }

    public void setArmSetpoint(double pos) {
        armPID.setSetpoint(pos);
    }

    public double getArmPosRad() {return 2 * Math.PI * armEncoder.getPosition();}
    public double getArmPos() {return armEncoder.getPosition();}

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
    public Command armToGround() {
        return armToPos(() -> ARM_GROUND_POSITION);
    }

    public Command armToTransfer() {
        return armToPos(() -> ARM_STOW_POSITION);
    }

    public Command setIntake(double speed) {
        return this.startEnd(() -> roller.set(speed), () -> roller.set(0));
    }

    public Command rollerIn() {return setIntake(ROLLER_INTAKE_SPEED);}
    public Command rollerOut() {return setIntake(ROLLER_OUTTAKE_SPEED);}
}
