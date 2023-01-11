package frc.lib.Ranging;

public class CargoBallDataPoint implements Comparable<CargoBallDataPoint> {

    // Variables
    private double distance;                // Distance calculated by Limelight sighting

    private double mainShooterSpeed;
    private double secondShooterSpeed;
    private double hoodPosition;
    private double realDistance;            // Real world distance

    public CargoBallDataPoint(double distance, double mainShooterSpeed, double secondShooterSpeed, 
        double hoodPosition, double realDistance) {
        // Save the Variables
        this.distance = distance;
        this.mainShooterSpeed = mainShooterSpeed;
        this.secondShooterSpeed = secondShooterSpeed;
        this.hoodPosition = hoodPosition;
        this.realDistance = realDistance;
    }

    public CargoBallDataPoint() {
        this(0.0, 0.0, 0.0, 0.0, 0.0);
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public double getDistance() {
        return distance;
    }

    public void setMainShooterSpeed(int speed) {
        this.mainShooterSpeed = speed;
    }

    public double getMainShooterSpeed() {
        return mainShooterSpeed;
    }

    public void setSecondShooterSpeed(int speed) {
        this.secondShooterSpeed = speed;
    }

    public double getSecondShooterSpeed() {
        return secondShooterSpeed;
    }

    public void setHoodPosition(double position) {
        this.hoodPosition = position;
    }

    public double getHoodPosition() {
        return hoodPosition;
    }

    public double getRealDistance() {
        return realDistance;
    }

    public void setRealDistance(double realDistance) {
        this.realDistance = realDistance;
    }

    @Override
    public int compareTo(CargoBallDataPoint dataPoint) {
        return Double.valueOf(distance).compareTo(dataPoint.getDistance());
    }
}
