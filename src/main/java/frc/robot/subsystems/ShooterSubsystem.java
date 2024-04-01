package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.LimelightConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends VerticalArm {
    private final CANSparkMax top;
    private final CANSparkMax bottom;
    private final CANSparkMax feeder;
    private final PIDController topPID;
    private final PIDController bottomPID;
    private final SimpleMotorFeedforward topFF;
    private final SimpleMotorFeedforward bottomFF;
    private double flywheelSetpoint = 0.0;


    public ShooterSubsystem() {
        super(new CANSparkMax(SHOOTER_ARM, CANSparkLowLevel.MotorType.kBrushless), ARM_INVERTED, ARM_PID, ARM_FF, 1, 12.0 / 48.0 / 28,  ARM_ABSOLUTE_OFFSET, false);
        top = new CANSparkMax(SHOOTER_TOP, CANSparkLowLevel.MotorType.kBrushless);
        bottom = new CANSparkMax(SHOOTER_BOTTOM, CANSparkLowLevel.MotorType.kBrushless);
        feeder = new CANSparkMax(SHOOTER_FEED, CANSparkLowLevel.MotorType.kBrushless);

        top.restoreFactoryDefaults();
        bottom.restoreFactoryDefaults();
        feeder.restoreFactoryDefaults();

        top.setIdleMode(CANSparkBase.IdleMode.kBrake);
        top.setIdleMode(CANSparkBase.IdleMode.kBrake);

        top.setInverted(TOP_INVERTED);
        bottom.setInverted(BOTTOM_INVERTED);
        feeder.setInverted(FEEDER_INVERTED);

        top.getEncoder().setVelocityConversionFactor(2.0);
        bottom.getEncoder().setVelocityConversionFactor(2.0);

        topPID = TOP_PID.getController();
        bottomPID = BOTTOM_PID.getController();
        topFF = TOP_FF.getSimpleFF();
        bottomFF = BOTTOM_FF.getSimpleFF();

    }

    @Override
    public void periodic() {
        super.periodic();
        setShooterControl(flywheelSetpoint);

        // Telemetry
        SmartDashboard.putNumber("Shooter Top RPM", getTopVelocityRPM());
        SmartDashboard.putNumber("Shooter Bottom RPM", getBottomVelocityRPM());
        SmartDashboard.putNumber("Shooter Top Setpoint", top.getAppliedOutput());
        SmartDashboard.putNumber("Shooter Bottom Setpoint", bottom.getAppliedOutput());
        SmartDashboard.putData("Shooter TopPID", topPID);
        SmartDashboard.putData("Shooter BottomPID", bottomPID);
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
    public Command armToAmp() {return armToPos(() -> AMP_POS);}
    public Command armToFeed() {return armToPos(() -> FEED_POS);}
    public Command armToStow() {return armToPos(() -> STOW_POS);}

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

    public Command getFeederCommand() {
        return runFeeder(FEEDER_SPEED);
    }

    public Command runShooter(double speed) {
        return new Command() {
            @Override
            public void initialize() {
                super.initialize();
                setShooter(speed);
            }

            @Override
            public void end(boolean interrupted) {
                //setShooter(0);
            }

            @Override
            public boolean isFinished() {
                return Math.abs(getTopVelocityRPM() - speed) < 150;
            }
        };
    }


    public Command stopShooter() {
        return runShooter(0);
    }

    /**
     * Align the shooter to the target using vision
     * @return
     */
    public Command alignShooter(VisionSubsystem vision) {
        return armToPos(() -> {
            if (vision.getNumTags() >= 2)
                return angleFromDistance(vision.getDistance());
            else return DEFAULT_SPEAKER_SHOT;
        });
    }

    private double angleFromDistance(double distance) {
        //assuming shooting in a straight line lol
        double error = 0.0; // speed decreases quadratically do smthing with that tycho
        return (Math.PI/2-Math.atan((SPEAKER_HEIGHT - LIMELIGHT_HEIGHT)/distance) + error)/2/Math.PI;
    }

    public Command scoreAmp(){
        return this.armToAmp()
                //.alongWith(drivebase.alignAmp())
                .andThen(
                        this.runShooter(AMP_RPM)
                                .alongWith(this.runFeeder(FEEDER_SPEED))
                                .withTimeout(1)
                                )
                .andThen(this.stopShooter());
    }

    public Command scoreShooter(){
        return this.runShooter(SHOOTER_RPM)
                .andThen(this.runFeeder(FEEDER_SPEED).withTimeout(1))
                .andThen(this.stopShooter());
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
