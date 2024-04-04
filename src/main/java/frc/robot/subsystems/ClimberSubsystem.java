package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimberConstants.*;

public class ClimberSubsystem extends SubsystemBase {
    // this is weird bc it uses shooter too
    // ig just winch

    private final CANSparkMax winch;

    public ClimberSubsystem() {
        this.winch = new CANSparkMax(WINCH, CANSparkLowLevel.MotorType.kBrushless);
        winch.restoreFactoryDefaults();

        winch.setIdleMode(CANSparkBase.IdleMode.kBrake);
        winch.setInverted(WINCH_INVERTED);
    }

    private void setWinch(double speed){
        this.winch.set(speed);
    }

    private void stopWinch(){
        this.winch.set(0); // always breaking but before it's unwound

    }

    public Command getWinchCommand(){
        return this.runEnd(() -> this.setWinch(1.0), this::stopWinch);
    }
    public Command getWinchReleaseCommand(){
    }
}
