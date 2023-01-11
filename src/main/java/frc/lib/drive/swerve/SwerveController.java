package frc.lib.drive.swerve;

public class SwerveController {

    // Swerve Variables
    private double L;
    private double W;
    private double r;

    public SwerveController(double length, double width) {
        // Swerve Variables
        L = length / 2.0;
        W = width / 2.0;
        r = Math.sqrt((L * L) + (W * W));
    }

    /**
     * @param x1 Joystick Strafe
     * @param y1 Joystick Speed
     * @param x2 Joystick Rotation
     */
    public double[] calculate(double x1, double y1, double x2) {
        // --------------------------------------
        // Swerve Module Math for Speed and Angle
        // --------------------------------------
        double a = x1 - x2 * (L / r);
        double b = x1 + x2 * (L / r);
        double c = y1 - x2 * (W / r);
        double d = y1 + x2 * (W / r);

        double frontLeftSpeed = Math.sqrt((b * b) + (c * c));
        double frontRightSpeed = Math.sqrt((b * b) + (d * d));
        double rearLeftSpeed = Math.sqrt((a * a) + (c * c));
        double rearRightSpeed = Math.sqrt((a * a) + (d * d));

        double frontLeftAngle = Math.atan2(b, c) * 180.0 / Math.PI;
        double frontRightAngle = Math.atan2(b, d) * 180.0 / Math.PI;
        double rearLeftAngle = Math.atan2(a, c) * 180.0 / Math.PI;
        double rearRightAngle = Math.atan2(a, d) * 180.0 / Math.PI;

        // -------------------------------------
        // Normalize the Speed
        // -------------------------------------
        double max = frontLeftSpeed;
        max = Math.max(max, frontRightSpeed);
        max = Math.max(max, rearLeftSpeed);
        max = Math.max(max, rearRightSpeed);

        if (max > 1) {
            frontRightSpeed /= max;
            frontLeftSpeed /= max;
            rearLeftSpeed /= max;
            rearRightSpeed /= max;
        }

        double[] stateArray = new double[8];

        stateArray[0] = frontLeftSpeed;
        stateArray[1] = frontLeftAngle;
        stateArray[2] = frontRightSpeed;
        stateArray[3] = frontRightAngle;
        stateArray[4] = rearLeftSpeed;
        stateArray[5] = rearLeftAngle;
        stateArray[6] = rearRightSpeed;
        stateArray[7] = rearRightAngle;

        return stateArray;
    }

    /**
     * @param x1   Joystick Strafe
     * @param y1   Joystick Speed
     * @param x2   Joystick Rotation
     * @param gyro Gyro Input (-180 to 180)
     */
    public double[] calculate(double x1, double y1, double x2, double gyroInput) {
        // Field Centric Code from NAVX Website
        double gyro = gyroInput * Math.PI / 180.0;

        double temp = x1 * Math.cos(gyro) + y1 * Math.sin(gyro);
        y1 = -x1 * Math.sin(gyro) + y1 * Math.cos(gyro);
        x1 = temp;

        // --------------------------------------
        // Swerve Module Math for Speed and Angle
        // --------------------------------------
        double a = x1 - x2 * (L / r);
        double b = x1 + x2 * (L / r);
        double c = y1 - x2 * (W / r);
        double d = y1 + x2 * (W / r);

        double frontLeftSpeed = Math.sqrt((b * b) + (c * c));
        double frontRightSpeed = Math.sqrt((b * b) + (d * d));
        double rearLeftSpeed = Math.sqrt((a * a) + (c * c));
        double rearRightSpeed = Math.sqrt((a * a) + (d * d));

        double frontLeftAngle = Math.atan2(b, c) * 180.0 / Math.PI;
        double frontRightAngle = Math.atan2(b, d) * 180.0 / Math.PI;
        double rearLeftAngle = Math.atan2(a, c) * 180.0 / Math.PI;
        double rearRightAngle = Math.atan2(a, d) * 180.0 / Math.PI;

        // -------------------------------------
        // Normalize the Speed
        // -------------------------------------
        double max = frontLeftSpeed;
        max = Math.max(max, frontRightSpeed);
        max = Math.max(max, rearLeftSpeed);
        max = Math.max(max, rearRightSpeed);

        if (max > 1) {
            frontRightSpeed /= max;
            frontLeftSpeed /= max;
            rearLeftSpeed /= max;
            rearRightSpeed /= max;
        }

        double[] stateArray = new double[8];

        stateArray[0] = frontLeftSpeed;
        stateArray[1] = frontLeftAngle;
        stateArray[2] = frontRightSpeed;
        stateArray[3] = frontRightAngle;
        stateArray[4] = rearLeftSpeed;
        stateArray[5] = rearLeftAngle;
        stateArray[6] = rearRightSpeed;
        stateArray[7] = rearRightAngle;

        return stateArray;
    }
}
