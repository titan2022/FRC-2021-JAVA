package frc.robot.motion.control;

public class PIDConfig {
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double CONTINOUS_MINIMUM = 0;
    public double CONTINOUS_MAXIMUM = 0;
    public double INTEGRATION_MAX = 0;
    public double INTEGRATION_MIN = 0;

    public PIDConfig(double kP, double kI, double kD, double CONTINOUS_MINIMUM, double CONTINOUS_MAXIMUM,
        double INTEGRATION_MIN, double INTEGRATION_MAX){
            kP = kP;
            kI = kI;
            kD = kD;
            CONTINOUS_MINIMUM = CONTINOUS_MINIMUM;
            CONTINOUS_MAXIMUM = CONTINOUS_MAXIMUM;
            INTEGRATION_MAX = INTEGRATION_MAX;
            INTEGRATION_MIN = INTEGRATION_MIN;
    }
}