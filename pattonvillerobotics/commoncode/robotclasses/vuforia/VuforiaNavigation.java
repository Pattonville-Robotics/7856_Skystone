package org.pattonvillerobotics.commoncode.robotclasses.vuforia;

import android.graphics.Bitmap;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

public class VuforiaNavigation {

    public static final String TAG = "VuforiaNavigation";
    public static final float MM_PER_INCH = 25.4f;

    private VuforiaTrackables targetsSkyStone;
    private VuforiaLocalizer vuforia;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackable relicTemplate;

    private OpenGLMatrix lastTrackedLocation;

    public VuforiaNavigation(VuforiaParameters parameters) {
        this.parameters = new VuforiaLocalizer.Parameters(parameters.getCameraMonitorViewId());
        this.parameters.cameraDirection = parameters.getCameraDirection();
        this.parameters.vuforiaLicenseKey = parameters.getLicenseKey();
        this.parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        this.parameters.useExtendedTracking = false;

        vuforia = ClassFactory.createVuforiaLocalizer(this.parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        setTrackableLocation(parameters.getTrackableLocations());
        setPhoneLocation(parameters.getPhoneLocation());

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        lastTrackedLocation = OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES, 0, 0, 0));
    }

    private void setTrackableLocation(List<OpenGLMatrix> locations) {
        for(int i = 0; i < locations.size(); i++) {
            targetsSkyStone.get(i).setLocation(locations.get(i));
        }
    }

    private void setPhoneLocation(OpenGLMatrix location) {
        for(VuforiaTrackable trackable : targetsSkyStone) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(location, parameters.cameraDirection);
        }
    }

    public void activateTracking() {
        if(targetsSkyStone != null) targetsSkyStone.activate();
    }

    public void deactivateTracking() {
        targetsSkyStone.deactivate();
    }

    public VuforiaTrackables getTrackables() {
        return targetsSkyStone;
    }

    private boolean trackableIsVisible(VuforiaTrackable trackable) {
        return ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible();
    }

    public RelicRecoveryVuMark getCurrentVisibleRelic() {
        return RelicRecoveryVuMark.from(relicTemplate);
    }

    public OpenGLMatrix getVisibleTrackableLocation() {
        for(VuforiaTrackable trackable : targetsSkyStone) {
            if(trackableIsVisible(trackable)) {
                OpenGLMatrix trackedLocation = ((VuforiaTrackableDefaultListener) trackable.getListener()).getRobotLocation();
                if(trackedLocation != null) lastTrackedLocation = trackedLocation;
                return lastTrackedLocation;
            }
        }
        return null;
    }



    public double getRobotX() {
        VectorF translation = lastTrackedLocation.getTranslation();
        return translation.get(0) / MM_PER_INCH;
    }

    public double getRobotY() {
        VectorF translation = lastTrackedLocation.getTranslation();
        return translation.get(1) / MM_PER_INCH;
    }

    public double getDistanceFromTrackable() {
        return Math.hypot(getRobotX(), getRobotY());
    }

    /**
     * @return Angle between robot and x axis
     */
    public double getRobotBearing() {
        return Orientation.getOrientation(lastTrackedLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    }

    /**
     * @return Angle between x axis and line formed between robot and trackable
     */
    public double getTrackableBearing() {
        return Math.toDegrees(-Math.asin(getRobotY()/getDistanceFromTrackable()));
    }

    /**
     * @return angle robot needs to rotate to face trackable
     */
    public double getAngleToTrackable() {
        return getTrackableBearing() - getRobotBearing();
    }

    public Bitmap getImage() {
        VuforiaLocalizer.CloseableFrame frame = null;
        Image rgbImage = null;

        try {
            frame = new VuforiaLocalizer.CloseableFrame(vuforia.getFrameQueue().take());
            for(int i = 0; i < frame.getNumImages(); i++) {
                Image img = frame.getImage(i);
                if(img.getFormat() == PIXEL_FORMAT.RGB565) {
                    rgbImage = img;
                    break;
                }
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        if(frame != null) frame.close();
        if(rgbImage != null) {
            Bitmap bm = Bitmap.createBitmap(rgbImage.getWidth(), rgbImage.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(rgbImage.getPixels());
            return bm;
        }
        return null;
    }
}
