
package frc.lib.vision;

import frc.lib.vision.LimeLight.CameraMode;
import frc.lib.vision.LimeLight.LedMode;

public class LimeLightManager {

    private LimeLight mLimelightCamera;

    private int ledOnRequest = 0;

    public LimeLightManager(LimeLight limeLightCamera) {
        mLimelightCamera = limeLightCamera;
    }

    public void ledModeRequest(LedMode ledMode) {
        switch (ledMode) {
        case On:
            ledOnRequest++;
            break;
        case Off:
            ledOnRequest--;
            break;
        case Blink:
            mLimelightCamera.setLedMode(LedMode.Blink);
            ledOnRequest = 0;
            break;
        case Pipeline:
            mLimelightCamera.setLedMode(LedMode.Pipeline);
            ledOnRequest = 0;
            break;
        }

        ledOnRequest = Math.max(0, ledOnRequest);

        if (ledOnRequest > 0) {
            mLimelightCamera.setLedMode(LedMode.On);
            mLimelightCamera.setCameraMode(CameraMode.Vision);
        } else {
            mLimelightCamera.setLedMode(LedMode.Off);
            mLimelightCamera.setCameraMode(CameraMode.Driver);
        }
    }
}
