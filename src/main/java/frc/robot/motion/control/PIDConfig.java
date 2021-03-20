package frc.robot.motion.control;

public class PIDConfig {
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double CONTINOUS_MINIMUM = 0;
    public double CONTINOUS_MAXIMUM = 0;
    public double INTEGRATION_MAX = 0;
    public double INTEGRATION_MIN = 0;

    public PIDConfig(){}

    public PIDConfig(double kP, double kI, double kD, double continousMinimum, double continousMaximum,
        double integrationMax, double integrationMin){
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.CONTINOUS_MINIMUM = continousMinimum;
            this.CONTINOUS_MAXIMUM = continousMaximum;
            this.INTEGRATION_MAX = integrationMax;
            this.INTEGRATION_MIN = integrationMin;
    }
}