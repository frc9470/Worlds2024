package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.ARM_TOLERANCE;
import static frc.robot.Constants.ShooterConstants.*;

import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
    private final CANSparkMax arm;
    private final CANSparkMax top;
    private final CANSparkMax bottom;
    private final CANSparkMax feeder;
    private final DutyCycleEncoder throughBore;
    private final PIDController armPID;
    private final RelativeEncoder armEncoder;
    private final ArmFeedforward armFF;

    public Shooter() {
        arm = new CANSparkMax(SHOOTER_ARM, CANSparkLowLevel.MotorType.kBrushless);
        top = new CANSparkMax(SHOOTER_TOP, CANSparkLowLevel.MotorType.kBrushless);
        bottom = new CANSparkMax(SHOOTER_BOTTOM, CANSparkLowLevel.MotorType.kBrushless);
        feeder = new CANSparkMax(SHOOTER_FEED, CANSparkLowLevel.MotorType.kBrushless);

        arm.restoreFactoryDefaults();
        top.restoreFactoryDefaults();
        bottom.restoreFactoryDefaults();
        feeder.restoreFactoryDefaults();

        arm.setInverted(ARM_INVERTED);
        top.setInverted(TOP_INVERTED);
        bottom.setInverted(BOTTOM_INVERTED);
        feeder.setInverted(FEEDER_INVERTED);

        throughBore = new DutyCycleEncoder(0);
        armEncoder = arm.getEncoder();
        armEncoder.setPositionConversionFactor(1.0 / 240.0);

        armPID = new PIDController(PID_CONSTANTS.kP, PID_CONSTANTS.kI, PID_CONSTANTS.kI, PID_CONSTANTS.iZone);
        armPID.setTolerance(ARM_TOLERANCE);
        armFF = new ArmFeedforward(ARM_KS, ARM_G, ARM_V, ARM_A);
        resetEncoder();
        setArmSetpoint(armEncoder.getPosition());
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
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //move the arm to setpoints every 20 ms
        arm.set(armPID.calculate(armEncoder.getPosition()) + armFF.calculate(getArmPosRad(), 0));
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

    public Command armToTrap() {return armToPos(()-> TRAP_POS);}
    public Command armToFeed() {return armToPos(()-> FEED_POS);}
}
