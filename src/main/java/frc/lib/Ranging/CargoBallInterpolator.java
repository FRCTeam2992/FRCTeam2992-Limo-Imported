package frc.lib.Ranging;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Handles creating list of setpoints and distances.
 * <p>
 * This class allows you to create list of setpoints and distances for use with
 * shooters. It can find the closest value for either the setpoint or distance.
 */
public class CargoBallInterpolator {

    private List<CargoBallDataPoint> dataPointList;

    public CargoBallInterpolator() {
        dataPointList = new ArrayList<CargoBallDataPoint>();
    }

    /**
     * @param distance the distance value.
     * @param setpoint the desired value for at the distance.
     */
    public void addDataPoint(CargoBallDataPoint dataPoint) {
        dataPointList.add(dataPoint);
        Collections.sort(dataPointList);
    }

    public void removeDataPoint(CargoBallDataPoint dataPoint) {
        dataPointList.remove(dataPoint);
    }

    /**
     * @param distance the distance value.
     * @return the desired value for at the distance.
     */
    public double calcMainShooterSpeed(double distance) {
        double tempMainSpeed = 0;
        if (dataPointList.size() == 1) {
            tempMainSpeed = dataPointList.get(0).getMainShooterSpeed();
        } else if (dataPointList.size() > 1) {
            CargoBallDataPoint upperDataPoint = new CargoBallDataPoint(-1.0, 0, 0, 0.0, 0.0);
            CargoBallDataPoint lowerDataPoint = new CargoBallDataPoint(-1.0, 0, 0, 0.0, 0.0);

            for (int i = 0; i < dataPointList.size(); i++) {
                if (dataPointList.get(i).getDistance() >= distance) {
                    upperDataPoint = dataPointList.get(i);

                    break;
                }

                lowerDataPoint = dataPointList.get(i);
            }
            if (lowerDataPoint.getDistance() == -1.0) {
                tempMainSpeed = upperDataPoint.getMainShooterSpeed();
            } else if (upperDataPoint.getDistance() == -1.0) {
                tempMainSpeed = lowerDataPoint.getMainShooterSpeed();
            } else {
                double upperMainSpeed = upperDataPoint.getMainShooterSpeed();
                double lowerMainSpeed = lowerDataPoint.getMainShooterSpeed();

                tempMainSpeed = lerp(lowerMainSpeed, upperMainSpeed, (distance - lowerDataPoint.getDistance())
                        / (upperDataPoint.getDistance() - lowerDataPoint.getDistance()));
            }
        }

        // SmartDashboard.putNumber("Main Shooter Speed", tempMainSpeed);

        return tempMainSpeed;
    }

    public double calcSecondShooterSpeed(double distance) {
        double tempSecondSpeed = 0;

        if (dataPointList.size() == 1) {
            tempSecondSpeed = dataPointList.get(0).getSecondShooterSpeed();
        } else if (dataPointList.size() > 1) {
            CargoBallDataPoint upperDataPoint = new CargoBallDataPoint(-1.0, 0, 0, 0.0, 0.0);
            CargoBallDataPoint lowerDataPoint = new CargoBallDataPoint(-1.0, 0, 0, 0.0, 0.0);

            for (int i = 0; i < dataPointList.size(); i++) {
                if (dataPointList.get(i).getDistance() >= distance) {
                    upperDataPoint = dataPointList.get(i);

                    break;
                }

                lowerDataPoint = dataPointList.get(i);
            }
            if (lowerDataPoint.getDistance() == -1.0) {
                tempSecondSpeed = upperDataPoint.getSecondShooterSpeed();
            } else if (upperDataPoint.getDistance() == -1.0) {
                tempSecondSpeed = lowerDataPoint.getSecondShooterSpeed();
            } else {
                double upperSecondSpeed = upperDataPoint.getSecondShooterSpeed();
                double lowerSecondSpeed = lowerDataPoint.getSecondShooterSpeed();

                tempSecondSpeed = lerp(lowerSecondSpeed, upperSecondSpeed, (distance - lowerDataPoint.getDistance())
                        / (upperDataPoint.getDistance() - lowerDataPoint.getDistance()));
            }
        }

        // SmartDashboard.putNumber("Second Shooter Speed", tempSecondSpeed);

        return tempSecondSpeed;
    }

    public double calcHoodPosition(double distance) {
        double tempHoodPosition = 0;

        if (dataPointList.size() == 1) {
            tempHoodPosition = dataPointList.get(0).getHoodPosition();
        } else if (dataPointList.size() > 1) {
            CargoBallDataPoint upperDataPoint = new CargoBallDataPoint(-1.0, 0, 0, 0.0, 0.0);
            CargoBallDataPoint lowerDataPoint = new CargoBallDataPoint(-1.0, 0, 0, 0.0, 0.0);

            for (int i = 0; i < dataPointList.size(); i++) {
                if (dataPointList.get(i).getDistance() >= distance) {
                    upperDataPoint = dataPointList.get(i);

                    break;
                }

                lowerDataPoint = dataPointList.get(i);
            }
            if (lowerDataPoint.getDistance() == -1.0) {
                tempHoodPosition = upperDataPoint.getHoodPosition();
            } else if (upperDataPoint.getDistance() == -1.0) {
                tempHoodPosition = lowerDataPoint.getHoodPosition();
            } else {
                double upperHoodPosition = upperDataPoint.getHoodPosition();
                double lowerHoodPosition = lowerDataPoint.getHoodPosition();

                tempHoodPosition = lerp(lowerHoodPosition, upperHoodPosition, (distance - lowerDataPoint.getDistance())
                        / (upperDataPoint.getDistance() - lowerDataPoint.getDistance()));
            }
        }

        // SmartDashboard.putNumber("Hood Position", tempHoodPosition);

        return tempHoodPosition;
    }

    public double calcRealDistance(double distance) {
        double tempRealDistance = 0;

        if (dataPointList.size() == 1) {
            tempRealDistance = dataPointList.get(0).getRealDistance();
        } else if (dataPointList.size() > 1) {
            CargoBallDataPoint upperDataPoint = new CargoBallDataPoint(-1.0, 0, 0, 0.0, 0.0);
            CargoBallDataPoint lowerDataPoint = new CargoBallDataPoint(-1.0, 0, 0, 0.0, 0.0);

            for (int i = 0; i < dataPointList.size(); i++) {
                if (dataPointList.get(i).getDistance() >= distance) {
                    upperDataPoint = dataPointList.get(i);

                    break;
                }

                lowerDataPoint = dataPointList.get(i);
            }
            if (lowerDataPoint.getDistance() == -1.0) {
                tempRealDistance = upperDataPoint.getRealDistance();
            } else if (upperDataPoint.getDistance() == -1.0) {
                tempRealDistance = lowerDataPoint.getRealDistance();
            } else {
                double upperRealDistance = upperDataPoint.getRealDistance();
                double lowerRealDistance = lowerDataPoint.getRealDistance();

                tempRealDistance = lerp(lowerRealDistance, upperRealDistance, (distance - lowerDataPoint.getDistance())
                        / (upperDataPoint.getDistance() - lowerDataPoint.getDistance()));
            }
        }

        // SmartDashboard.putNumber("Real Distance", tempRealDistance);

        return tempRealDistance;
    }

    private double lerp(double start, double end, double count) {
        return start + (count * (end - start));
    }

    // private int lerp(int start, int end, double count) {
    //     return (int) Math.round(start + (count * (end - start)));
    // }
}