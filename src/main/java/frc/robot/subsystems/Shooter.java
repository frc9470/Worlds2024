package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends VerticalArm {
    private final CANSparkMax top;
    private final CANSparkMax bottom;
    private final CANSparkMax feeder;
    private final PIDController topPID;
    private final PIDController bottomPID;
    private final SimpleMotorFeedforward topFF;
    private final SimpleMotorFeedforward bottomFF;
    private double flywheelSetpoint = 0.0;


    public Shooter() {
        super(new CANSparkMax(SHOOTER_ARM, CANSparkLowLevel.MotorType.kBrushless), ARM_PID, ARM_FF, 1, 1 / 48.0 / 5.0, 0);
        top = new CANSparkMax(SHOOTER_TOP, CANSparkLowLevel.MotorType.kBrushless);
        bottom = new CANSparkMax(SHOOTER_BOTTOM, CANSparkLowLevel.MotorType.kBrushless);
        feeder = new CANSparkMax(SHOOTER_FEED, CANSparkLowLevel.MotorType.kBrushless);

        top.restoreFactoryDefaults();
        bottom.restoreFactoryDefaults();
        feeder.restoreFactoryDefaults();

        arm.setInverted(ARM_INVERTED);
        top.setInverted(TOP_INVERTED);
        bottom.setInverted(BOTTOM_INVERTED);
        feeder.setInverted(FEEDER_INVERTED);

        topPID = TOP_PID.getController();
        bottomPID = BOTTOM_PID.getController();
        topFF = TOP_FF.getSimpleFF();
        bottomFF = BOTTOM_FF.getSimpleFF();

    }

    @Override
    public void periodic() {
        super.periodic();
        setShooterControl(flywheelSetpoint);
    }

    public double getTopVelocityRPM() {
        return top.getEncoder().getVelocity();
    }

    public double getBottomVelocityRPM() {
        return bottom.getEncoder().getVelocity();
    }



    public void setTop(double volts) {
        top.setVoltage(volts);
    }

    public void setBottom(double volts) {
        bottom.setVoltage(volts);
    }

    private void setShooterControl(double topSpeed, double bottomSpeed) {
        setTop(topPID.calculate(getTopVelocityRPM(), topSpeed) + topFF.calculate(topSpeed));
        setBottom(bottomPID.calculate(getBottomVelocityRPM(), bottomSpeed) + bottomFF.calculate(bottomSpeed));
    }

    private void setShooterControl(double speed){
        setShooterControl(speed, speed);
    }

    public void setShooter(double speed) {
        this.flywheelSetpoint = speed;
    }


    public void setFeeder(double speed) {
        feeder.set(speed);
    }

     /*********************************************************************************
     ********************************* COMMANDS **************************************
     *********************************************************************************/
    public Command armToAmp() {return armToPos(()-> AMP_POS);}
    public Command armToFeed() {return armToPos(()-> FEED_POS);}

    public Command runFeeder(double speed) {
        return new Command() {
            @Override
            public void initialize() {
                super.initialize();
                feeder.set(speed);
            }

            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                feeder.set(0);
            }
        };
    }

    public Command runShooter(double speed) {
        return new Command() {
            @Override
            public void initialize() {
                super.initialize();
                setShooter(speed);
            }

            @Override
            public boolean isFinished() {
                return Math.abs(getTopVelocityRPM() - speed) < 100;
            }
        };
    }

    public Command stopShooter() {
        return runShooter(0);
    }

}
