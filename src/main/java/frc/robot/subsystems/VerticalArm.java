package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FFConstants;
import frc.robot.util.PIDConstants;

import java.util.function.Supplier;

public abstract class VerticalArm extends SubsystemBase {
    protected final CANSparkMax arm;
    protected final DutyCycleEncoder throughBore;
    protected final RelativeEncoder armEncoder;
    protected final PIDController armPID;
    protected final ArmFeedforward armFF;
    private double offset;

    public VerticalArm(CANSparkMax arm, boolean inverted, PIDConstants pidConstants, FFConstants armFF, int encoderPort, double ratio, double offset) {
        this.arm = arm;
        arm.restoreFactoryDefaults();
        arm.setIdleMode(CANSparkMax.IdleMode.kBrake);
        arm.setInverted(inverted);
        arm.setSmartCurrentLimit(40);

        this.armPID = pidConstants.getController();
        this.armFF = armFF.getArmFF();

        throughBore = new DutyCycleEncoder(encoderPort);
        armEncoder = arm.getEncoder();
        armEncoder.setPositionConversionFactor(ratio);

        this.offset = offset;
        resetEncoder();

        setArmSetpoint(armEncoder.getPosition() * ratio);
    }

    public void resetEncoder() {
        armEncoder.setPosition(throughBore.getAbsolutePosition() + offset);
    }

    public void setArmSetpoint(double pos) {
        if (pos > 1 || pos < -1) throw new IllegalArgumentException("Error: Arm Setpoint " + pos + " for " + this.getName() + " outside safety bounds.");
        pos = MathUtil.clamp(pos, -0.2, 0.5);
        armPID.setSetpoint(pos);
    }

    public double getArmPosRad() {
        return 2 * Math.PI * getArmPos();
    }

    public double getArmPos() {
        return armEncoder.getPosition();
    }

    @Override
    public void periodic() {
        //move the arm to the setpoint every 20 ms
        // check setpoint and clamp it to -0.2, 0.5
        double setpoint = armPID.getSetpoint();
        setpoint = MathUtil.clamp(setpoint, -0.2, 0.5);
        if (setpoint != armPID.getSetpoint()) System.out.println("Warning: Illegal arm setpoint clamped. " + armPID.getSetpoint());
        setArmSetpoint(setpoint);

        double output = armPID.calculate(getArmPos()) + armFF.calculate(getArmPosRad(), 0);
        arm.set(output);
        SmartDashboard.putNumber(this.getName() + " Position", getArmPos());
        SmartDashboard.putNumber(this.getName() + " Error", getArmPos() - armPID.getSetpoint());
        SmartDashboard.putNumber(this.getName() + " Rotations", armPID.getSetpoint());
        SmartDashboard.putNumber(this.getName() + " Arm Output", output);
        SmartDashboard.putNumber(this.getName() + " Count", SmartDashboard.getNumber("Count", 0) + 1);
        SmartDashboard.putNumber(this.getName() + " Absolute Encoder Position", throughBore.getAbsolutePosition());
        SmartDashboard.putData(this.getName() + " PID", armPID);
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
/*
⠀⠀⠀⠀⠀⠀⠀⠀⢠⣾⣯⣟⡆⢸⢠⣯⡃⠀⠀⠀⠀⢹⣆⠀⠀⠀⠁⠀⠀⠀⠀⠀⠀⠀⢈⣿⠉⢹⢉⢹⡌⢉⡜⠚⠭⠤⠋⠑⣼⣿⢟⡿⣿⣻⣯⣸⡇⣿⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⣦⡄⠀⠀⠀⠀⢸⡿⢄⣈⠀⢸⣿⣿⠃⠀⠀⠀⠀⡎⣿⡄⠀⠀⠀⠀⠀⢀⣀⣤⣶⣾⣿⠟⠙⢸⠯⣈⠋⠉⠀⣼⣾⣦⣤⣶⣿⣶⡟⢼⣺⣝⠾⠕⠙⠻⣽⣖⣤⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠘⡟⠙⢦⡀⠀⠖⠛⠛⠺⠶⣉⢦⣷⡇⠀⠀⠀⠀⠀⠸⡸⣷⡆⣀⣤⣴⣾⡿⠟⠛⠉⠀⠈⠛⠛⠻⢦⣼⣶⣶⣾⣿⣿⣿⣿⣿⣿⣿⣷⣖⣬⣴⢷⣶⡴⣶⣿⣦⣤⡙⠻⣶⣄⠀⠀⠀⠀⠀⠀⠀
⠀⠈⠀⠀⠀⠹⢦⡀⠀⠀⠀⠀⠀⠙⢿⣿⡄⠀⠀⠀⠀⠀⢣⢻⣿⡿⠖⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠙⠻⢿⣿⣿⣿⣿⣿⣿⣿⡏⣾⣿⣿⡜⢿⣿⣮⢻⣿⣿⣿⣷⣾⢻⡆⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠈⠙⢦⡀⠀⠀⠀⠀⠀⠙⢷⡀⠀⠀⢀⣠⣼⣿⡟⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣤⣴⣶⠿⠟⠛⠋⠉⠁⠀⣼⢰⡿⢿⣿⣿⣮⣛⢿⡇⣿⣿⣿⢿⣷⡿⠁⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢦⡀⠀⠀⠀⠀⠀⣳⣰⣾⣿⣿⣿⣿⠃⠀⠀⠀⠀⠀⠀⠀⣀⣤⣴⣶⣿⣿⣿⣍⣄⣀⣀⣀⡐⣀⣠⣠⣄⣿⢸⣿⣿⣿⣿⣿⣯⡟⣰⣷⣿⣿⣿⠟⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢦⡀⢠⣴⣿⣿⣿⣿⣿⣿⣿⡏⠀⠀⠀⢀⣠⣴⣾⣿⣿⣿⣿⣿⠿⠿⠿⠿⢿⣿⡛⢟⠛⠛⠛⠛⠛⠿⣿⡹⣻⠷⢿⣛⣭⣾⠟⢿⣿⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣾⠿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠁⣠⣴⣾⣿⣿⣿⣿⡿⠟⠋⠁⠀⠀⠀⠀⠀⠀⠉⡻⣮⡳⣄⠀⠀⠀⠀⠨⡻⡻⡟⢻⣿⡿⣇⠀⠈⢿⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠁⠀⠀⠀⠀⠀⠀⠀⢀⣠⣾⠟⠙⠁⠙⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠟⠋⠁⠀⠀⠀⠀⠀⠀⢀⡠⠀⠤⠤⠐⠌⢿⣾⣦⡀⠀⠀⠀⢳⡙⢟⡄⠹⡿⡇⡆⠀⠈⣿⣷⣄⠀⠀⠀⠀⠀⠀⠀⠀
⣌⠢⣄⠀⠀⠀⠀⣰⣿⢿⠃⠀⠀⠀⠐⢿⣿⣿⣿⣿⠟⠋⣁⣾⣿⡟⠉⠀⠀⠀⠀⠀⢀⡈⠲⢜⡛⠥⠄⠖⠉⠁⢀⣀⣀⣘⢳⣵⡄⠀⠀⠀⢷⠘⣽⡄⢱⡇⢣⠀⠀⠘⣿⣿⣿⣦⡀⠀⠀⠀⠀⠀
⠈⠳⣌⠳⣄⣤⣾⠟⢡⠏⠀⠀⠀⠀⢀⣬⡿⠟⠉⠀⠀⣠⡿⠋⠘⢿⡦⣄⠀⢀⠤⠐⠁⠀⠀⠀⠹⢶⣤⣶⠾⠛⠉⠁⠀⢀⣰⠙⣿⡄⠀⠀⠘⡇⢸⢷⠘⣷⠀⠀⠀⠀⢹⣿⣿⣿⣿⣷⣄⡀⠀⠀
⠀⠀⠈⠳⣼⡽⠋⢀⡞⠀⠀⠀⣠⣴⠟⠉⢀⡄⠀⢀⣴⡋⠀⠀⠀⡄⠙⠮⣷⣤⡤⠜⠁⠀⠀⣠⡾⠟⠉⠛⠿⣿⣭⠿⠿⣿⣿⣿⣽⣷⡀⠀⠀⣽⠸⣛⣆⡇⠀⠀⠀⠀⠀⢿⣿⣧⡙⣿⣿⣿⣶⣤
⠀⠀⠀⠀⡿⠀⠀⣼⠁⢀⣤⡾⠋⠀⠀⠀⠘⣿⣤⠞⠀⠙⢦⣄⢸⠁⠀⠀⠈⠑⠿⣶⣤⣤⣾⠋⠀⠀⣀⣀⣀⠁⠈⠙⢒⣦⣤⣼⣥⣬⢷⡀⠀⢸⠀⣿⣿⡇⠀⠀⠀⠀⠀⠸⣷⠙⢿⣌⠹⠟⠙⠻
⠀⠀⠀⢰⠃⠀⣸⢁⣴⣿⠋⠀⠀⠀⠀⠀⠀⡾⣷⡀⣀⢀⡴⠙⢷⣍⠢⢄⠏⠉⠙⠮⠉⠻⢤⡛⠉⠉⠉⠁⢀⣬⣭⣿⣿⣿⣿⠿⢿⣿⡘⢧⠀⢸⣸⡏⣿⠁⠀⢠⠀⠀⠀⡇⢻⡤⠀⠙⣷⣄⠀⠀
⠀⠀⢀⡏⢀⣼⣵⣿⣿⣿⡀⠀⠀⠀⠀⠀⡼⠁⢈⢿⡆⠑⠦⣀⠀⣙⣻⢄⡀⠀⠀⠀⠀⠀⠀⠉⠳⢦⣀⣞⣵⣿⣿⣫⡯⡭⡿⢷⠀⣿⠃⠸⡇⠘⣿⢧⡏⠀⠀⠏⠀⣀⣀⢧⠸⣧⡀⠀⠀⠙⢷⡔
⠀⠀⣸⢡⣮⣾⣿⣿⣿⡇⣿⡄⠀⠀⠀⣸⠁⣠⠏⡼⣿⣶⣿⣂⣥⡤⡄⠀⠉⠛⢿⣵⣦⣀⣀⣀⡀⣀⣈⣙⣻⢿⣣⡮⡅⠈⠛⢽⣧⡏⠀⠀⣷⣼⡿⣾⡁⠀⠐⠀⠀⠀⠀⢈⣱⡇⠙⢷⣤⡀⠀⠉
⠀⢠⣧⣿⣿⣿⣿⣿⣿⣇⡇⢷⠀⠀⣸⢻⡂⠃⡰⠁⣧⣷⠟⠻⣕⡦⢌⣑⠲⠤⣄⣈⠙⠢⢤⣀⡀⠀⢸⠏⣠⠈⠸⣟⣎⣄⡴⠚⠁⠀⠀⢀⣿⠟⣧⡿⠛⣶⣦⠤⠤⠶⣾⠛⠉⠀⠀⠀⠻⣟⠳⢤
⠀⡜⣾⣿⣿⣿⣿⣿⣿⣿⢹⢻⣇⢠⡏⠀⢳⡐⠃⣰⠏⣷⠀⠀⠀⠙⠳⠮⣏⡲⠒⠭⠭⠍⣒⡒⡻⠿⣟⣋⣁⣀⠤⠀⠋⠁⠀⠀⠀⠀⠤⠞⢁⣴⠟⣁⡼⠋⠈⣇⠀⠀⠙⣆⠀⠀⠀⠀⠀⠙⢧⡀
⠀⠻⣛⠿⣿⣿⣿⣿⣿⣿⡏⡎⢿⣾⣱⡔⣌⢷⣜⠁⠀⡇⡂⠀⠀⣠⣴⣒⣒⣉⣙⠓⠒⠒⠤⠴⠔⠛⠛⠋⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠠⠴⣿⣡⣾⣿⡀⠀⠀⢹⠀⠀⠀⢹⣆⠀⠀⠀⠀⠀⠈⢳
⠀⠀⠈⠉⠐⠚⠫⠽⠿⠿⢧⠸⡜⣾⣇⠹⡜⢮⢳⡵⠄⣇⠡⢤⣾⣿⣿⣿⡿⣿⣯⡉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠤⠴⠟⠋⠁⡿⣿⣧⠀⠀⠸⣿⠐⣰⢻⣿⡆⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡀⣷⡈⢿⡆⠙⢦⡳⣿⣦⣻⡤⣿⡿⠻⣗⣤⣯⡀⠻⢷⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⠃⢿⣹⡄⠀⠀⣿⠀⣿⠀⡟⣿⡀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⣇⢷⢳⡀⠙⢤⠀⠉⠚⠻⣿⣾⢿⡧⣀⠈⠺⣷⣇⣠⠞⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢰⠏⠀⢸⡏⢷⠀⠀⣿⡄⢹⠀⢧⠸⣇⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⡼⡆⠳⡄⠀⢕⢤⡀⠀⠈⠻⢷⣍⡑⠋⠶⠲⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⠔⡿⠀⠀⠀⠀⠀⣠⡏⠀⠀⠀⣿⣜⣧⢠⡏⠁⠸⠀⠈⠇⣻⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢻⣿⡀⠙⢦⡀⠙⢿⣶⣤⠀⠀⠉⠻⣦⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⡤⠒⠉⢀⡼⠁⠀⠀⠀⠀⣴⠋⠀⣀⡴⡞⠛⣿⡼⣾⡇⠀⠀⠀⠀⢸⠘⡧⡀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠻⣿⣰⡷⠛⢦⡈⠓⢍⠻⣶⢦⣤⣌⣻⣷⣦⣤⡤⠀⠀⠀⠀⠀⠀⠀⠉⠒⠒⠋⠁⠀⠀⠀⠀⢀⣴⠃⣠⡾⠋⠁⠀⠀⢻⣿⣾⠃⠀⡅⠀⠀⣼⡀⣷⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣼⡿⢁⠀⠈⣿⢲⣤⢕⡺⣷⣇⢸⠀⠀⢸⡲⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡞⣡⡾⠉⠀⠀⠀⠀⠀⠈⢻⣿⡆⢰⠁⠀⡴⡏⡇⢹⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣖⡿⠃⠀⠀⠀⢀⣾⠀⠈⡟⠺⣭⣟⣻⢤⠀⠀⡇⣿⠋⠙⢶⣶⡤⠤⠤⠤⣤⣤⣀⣀⣀⣀⡴⠋⣼⠟⠀⠀⠀⠀⠀⠀⠀⣠⣾⡇⡸⡟⠀⣘⡰⡇⠇⢸⡇⠀⠀⠀
⠠⠤⣚⣛⠷⣤⠀⠀⣀⢤⣺⣿⡏⠀⠀⠀⢀⡴⠋⢸⠀⠀⠳⢸⠃⢸⢹⡉⠉⠉⠀⢸⠀⠀⠈⣿⡀⠀⠀⠀⠀⠀⠈⣿⣧⡀⢀⡾⠃⠀⠀⠀⠀⠀⠀⢀⣾⣿⡿⣄⡿⡴⠉⠀⠀⡇⠀⢸⣷⣶⣶⣤
⠄⠀⠉⠉⢉⣉⣩⣭⣷⣿⣿⣿⣅⣀⡤⠚⠁⠀⠀⣼⠀⠀⠀⣼⠀⣼⠐⡇⠀⢀⠀⣾⠀⠀⠀⡇⡇⠀⠀⠀⠀⣠⣿⣿⣿⣷⡞⠁⠀⠀⠀⠀⠀⠀⣰⣿⣿⣿⢃⡹⡼⠀⠀⠀⠀⣿⠀⣼⣿⣿⡿⠛
⠃⠀⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠀⠀⠀⠀⠀⠀⠀⡏⠀⠀⢀⡏⠀⡟⡀⢷⠀⡾⢀⡿⠀⠀⠀⢡⢸⠀⣀⣴⣾⣿⣿⣿⣿⣿⣗⣪⡖⢶⡆⠀⢀⣾⣿⣿⣿⠏⢡⠳⠁⠀⠀⠀⢰⢻⣠⠇⣿⠟⠁⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⠃⠀⢀⡾⠀⢠⣷⡇⠸⡶⡇⣸⠃⠀⠀⠀⣘⣼⣾⣿⣿⣿⣿⡿⣻⣛⣿⣭⡿⢷⠾⠃⣠⣾⣿⠿⢫⡏⢀⡏⠀⠀⠀⠀⢀⡪⣿⠸⢸⡇⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⠏⠀⣠⠟⠀⠀⣼⣾⢱⡀⢻⣡⡏⠀⠀⣰⣿⣿⣿⣿⣿⣿⣿⠃⠻⠟⣹⣿⣷⠀⢀⣴⡿⣿⠟⠁⣤⠟⢀⡞⠀⠀⠀⠀⠔⠋⢠⡇⠁⣾⠇⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠟⣠⠾⠃⠀⠀⢠⡇⣿⣈⢧⠈⢿⣇⣴⣿⣿⣿⣿⣿⣿⢟⣿⠇⠀⢀⣼⣿⣿⣿⣷⠿⣫⠞⠁⣠⣾⠋⢠⡞⠀⠀⠀⠀⠀⠀⠀⣼⡅⢠⣿⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⣾⠵⠊⠁⠀⠀⠀⠀⣼⢱⣿⡈⢪⢧⡈⢿⣱⣿⣿⣿⣿⣿⢫⢸⣿⠀⣤⠟⠋⢡⣴⠟⡡⠊⢡⡠⣾⠟⢁⡴⠛⠀⠀⠀⠀⠀⢀⠀⣰⡟⢀⣼⡏⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡟⢸⡿⡇⠀⠈⢳⡈⢿⣿⣿⣿⠋⠀⢸⠈⣿⡟⠂⣠⡾⢋⡴⠊⠀⠚⣻⠞⢁⣴⠟⢀⠔⠁⠀⠀⠀⢀⠌⣴⣿⡡⠾⠋⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⡇⠀⡇⢳⠀⠀⠀⠹⢄⢻⡟⠁⠀⠀⢸⠀⢻⣧⡾⢋⣐⣋⣀⣬⣔⣯⠴⢺⡟⠁⠀⠀⠀⠀⠀⢀⡠⠮⢞⡟⠉⠘⠁⠀⠀⠀⠀⠀⠀
*/
