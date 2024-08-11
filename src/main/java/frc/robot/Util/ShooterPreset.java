package frc.robot.Util;

/**
 * Shooter preset which is used in the vision lookup table
 */
public class ShooterPreset implements Comparable<ShooterPreset> {
    private double armAngle;
    private double leftShooterSpeed;
    private double rightShooterSpeed;
    private double distance;

    public ShooterPreset(double pArmAngle,
                         double pLeftShooterSpeed,
                         double pRightShooterSpeed,
                         double pDistance){
        this.armAngle = pArmAngle;
        this.leftShooterSpeed = pLeftShooterSpeed;
        this.rightShooterSpeed = pRightShooterSpeed;
        this.distance = pDistance;
    }

    public double getArmAngle() {
        return armAngle;
    }

    public double getLeftShooterSpeed(){
        return leftShooterSpeed;
    }

    public double getRightShooterSpeed(){
        return rightShooterSpeed;
    }

    public double getDistance(){
        return distance;
    }


    public void setArmAngle(double pArmAngle) {
        this.armAngle = pArmAngle;
    }

    public void setLeftShooterSpeed(double pLeftShooterSpeed) {
        this.leftShooterSpeed = pLeftShooterSpeed;
    }

    public void setRightShooterSpeed(double pRightShooterSpeed) {
        this.rightShooterSpeed = pRightShooterSpeed;
    }

    public void setDistance(double pDistance) {
        this.distance = pDistance;
    }


    @Override
    public int compareTo(ShooterPreset pPreset) {
        return Double.compare(this.getDistance(), pPreset.getDistance());
    }
}
