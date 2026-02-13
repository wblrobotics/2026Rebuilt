package frc.robot.lib.motors.positionController;

public class PIDConfig {
    private double kP;
    private double kI;
    private double kD;
    private double kS;
    private double kV;
    private double kA;

    public PIDConfig(double kP, double kI, double kD, double kS, double kV, double kA){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }
    
    public double getkP(){
        return kP;
    }

    public double getkI(){
        return kI;
    }

    public double getkD(){
        return kD;
    }

    public double getkS(){
        return kS;
    }

    public double getkV(){
        return kV;
    }
    
    public double getkA(){
        return kA;
    }
}
