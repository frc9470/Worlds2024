package frc.robot.util;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public record FFConstants(double kS, double kG, double kV, double kA) {

    public FFConstants(double kS, double kV, double kA) {
        this(kS, 0.0, kV, kA);
    }

    public FFConstants(double kS, double kV) {
        this(kS, 0.0, kV, 0.0);
    }

    public SimpleMotorFeedforward getSimpleFF() {
        return new SimpleMotorFeedforward(kS, kV, kA);
    }

    public ArmFeedforward getArmFF() {
        return new ArmFeedforward(kS, kG, kV, kA);
    }
}
