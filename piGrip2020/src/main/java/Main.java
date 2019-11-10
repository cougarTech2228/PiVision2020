
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import java.util.HashMap;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.VideoMode.PixelFormat;   
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.vision.VisionThread;

import org.opencv.core.*;
import org.opencv.imgproc.*;

/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ],
               "stream": {                              // optional
                   "properties": [
                       {
                           "name": <stream property name>
                           "value": <stream property value>
                       }
                   ]
               }
           }
       ]
   }
 */

// **************************************************************************
// * 
// * Main Class
// *
// **************************************************************************
public final class Main {

  public static final int MJPEG_OPENCV_SERVER_PORT = 1183;
  public static final double IMAGE_WIDTH_PIXELS = 320.0;
  public static final double IMAGE_HEIGHT_PIXELS = 240.0;
  public static final int DEFAULT_FRAME_RATE = 30;
  public static final double HALF_IMAGE_WIDTH_IN_PIXELS = IMAGE_WIDTH_PIXELS / 2.0;

  public static final int TARGETING_STATE_SEARCHING = 0;
  public static final int TARGETING_STATE_ACQUIRING = 1;
  public static final int TARGETING_STATE_LOCKED = 2;

  public static final double TARGET_HEIGHT_INCHES = 5.5;
  public static final double TARGET_WIDTH_INCHES = 2.0;

  public static final double TARGET_ASPECT_RATIO_TOLERANCE = .20;
  public static final double TARGET_ASPECT_RATIO_FOR_LOW_ANGLE = TARGET_HEIGHT_INCHES / TARGET_WIDTH_INCHES;
  public static final double TARGET_ASPECT_RATIO_MIN_THRESHOLD_FOR_LOW_ANGLE = TARGET_ASPECT_RATIO_FOR_LOW_ANGLE
      - (TARGET_ASPECT_RATIO_FOR_LOW_ANGLE * TARGET_ASPECT_RATIO_TOLERANCE);
  public static final double TARGET_ASPECT_RATIO_MAX_THRESHOLD_FOR_LOW_ANGLE = TARGET_ASPECT_RATIO_FOR_LOW_ANGLE
      + (TARGET_ASPECT_RATIO_FOR_LOW_ANGLE * TARGET_ASPECT_RATIO_TOLERANCE);

  // For the high angle (i.e., the ~-75 degree vision strip), we need to swap the
  // height and width in the aspect ratio calculation. TODO - not sure why this is
  // necessary
  public static final double TARGET_ASPECT_RATIO_FOR_HIGH_ANGLE = TARGET_WIDTH_INCHES / TARGET_HEIGHT_INCHES;
  public static final double TARGET_ASPECT_RATIO_MIN_THRESHOLD_FOR_HIGH_ANGLE = TARGET_ASPECT_RATIO_FOR_HIGH_ANGLE
      - (TARGET_ASPECT_RATIO_FOR_HIGH_ANGLE * TARGET_ASPECT_RATIO_TOLERANCE);
  public static final double TARGET_ASPECT_RATIO_MAX_THRESHOLD_FOR_HIGH_ANGLE = TARGET_ASPECT_RATIO_FOR_HIGH_ANGLE
      + (TARGET_ASPECT_RATIO_FOR_HIGH_ANGLE * TARGET_ASPECT_RATIO_TOLERANCE);

  public static final double TARGET_LOW_ANGLE = -15.0;
  public static final double TARGET_HIGH_ANGLE = -75.0;
  public static final double TARGET_ANGLE_TOLERANCE_IN_DEGREES = 10;
  public static final double TARGET_LOW_ANGLE_MIN_THRESHOLD = TARGET_LOW_ANGLE - TARGET_ANGLE_TOLERANCE_IN_DEGREES;
  public static final double TARGET_LOW_ANGLE_MAX_THRESHOLD = TARGET_LOW_ANGLE + TARGET_ANGLE_TOLERANCE_IN_DEGREES;
  public static final double TARGET_HIGH_ANGLE_MIN_THRESHOLD = TARGET_HIGH_ANGLE - TARGET_ANGLE_TOLERANCE_IN_DEGREES;
  public static final double TARGET_HIGH_ANGLE_MAX_THRESHOLD = TARGET_HIGH_ANGLE + TARGET_ANGLE_TOLERANCE_IN_DEGREES;

  public static final int MIN_HASH_MAP_DISTANCE = 18;
  public static final int MAX_HASH_MAP_DISTANCE = 48;

  public static final double MINIMUM_HORIZONTAL_OFFSET_REQ_IN_PIXELS = 200.0;

  // When we were empirically collecting data for the distance calculation hash
  // map,
  // we observed that the actual measured distance between the front of the camera
  // and the target was roughly 2 inches less than what the distance calculation
  // in
  // the code was telling us.
  public static final double DISTANCE_CORRECTION_OFFSET = 2.0;

  // !!! Very small changes in this constant dramatically affects distance calc
  // accuracy !!!
  public static final double CAMERA_FOV_ANGLE = 60.010; // FOV Angle determined empirically
  public static final double CAMERA_FOV_ANGLE_CALC = Math.tan(CAMERA_FOV_ANGLE);

  private static String configFile = "/boot/frc.json";

  public static class CameraConfig {
    public String name;
    public String path;
    public JsonObject config;
    public JsonElement streamConfig;
  }

  public static int team;
  public static boolean server;
  public static List<CameraConfig> cameraConfigs = new ArrayList<>();
  public static int targetingState = TARGETING_STATE_SEARCHING;

  // This will be the list of targets that we'll use to determine whether or not
  // we're locked on the two angle vision tape strips.
  public static List<Rect> targets = new ArrayList<>();
  public static List<RotatedRect> targetRects = new ArrayList<>();

  private Main() {
  }

  // **************************************************************************
  // *
  // * Report parse error.
  // *
  // **************************************************************************
  public static void parseError(String str) {
    System.err.println("config error in '" + configFile + "': " + str);
  }

  // **************************************************************************
  // *
  // * Read single camera configuration
  // *
  // **************************************************************************
  public static boolean readCameraConfig(JsonObject config) {
    CameraConfig cam = new CameraConfig();

    // name
    JsonElement nameElement = config.get("name");
    if (nameElement == null) {
      parseError("could not read camera name");
      return false;
    }
    cam.name = nameElement.getAsString();

    // path
    JsonElement pathElement = config.get("path");
    if (pathElement == null) {
      parseError("camera '" + cam.name + "': could not read path");
      return false;
    }
    cam.path = pathElement.getAsString();

    // stream properties
    cam.streamConfig = config.get("stream");

    cam.config = config;

    cameraConfigs.add(cam);
    return true;
  }

  // **************************************************************************
  // *
  // * Read configuration file.
  // *
  // **************************************************************************
  public static boolean readConfig() {
    // parse file
    JsonElement top;

    try {
      top = new JsonParser().parse(Files.newBufferedReader(Paths.get(configFile)));
    } catch (IOException ex) {
      System.err.println("could not open '" + configFile + "': " + ex);
      return false;
    }

    // top level must be an object
    if (!top.isJsonObject()) {
      parseError("must be JSON object");
      return false;
    }

    JsonObject obj = top.getAsJsonObject();

    // team number
    JsonElement teamElement = obj.get("team");

    if (teamElement == null) {
      parseError("could not read team number");
      return false;
    }

    team = teamElement.getAsInt();

    // ntmode (optional)
    if (obj.has("ntmode")) {

      String str = obj.get("ntmode").getAsString();

      if ("client".equalsIgnoreCase(str)) {
        server = false;
      } else if ("server".equalsIgnoreCase(str)) {
        server = true;
      } else {
        parseError("could not understand ntmode value '" + str + "'");
      }
    }

    JsonElement camerasElement = obj.get("cameras");

    if (camerasElement == null) {
      parseError("could not read cameras");
      return false;
    }

    JsonArray cameras = camerasElement.getAsJsonArray();

    for (JsonElement camera : cameras) {
      if (!readCameraConfig(camera.getAsJsonObject())) {
        return false;
      }
    }

    return true;
  }

  // **************************************************************************
  // *
  // * Start running the camera
  // *
  // **************************************************************************
  public static VideoSource startCamera(CameraConfig config) {
    System.out.println("Starting camera '" + config.name + "' on " + config.path);
    UsbCamera camera = new UsbCamera(config.name, config.path);
    MjpegServer mjpegServer = CameraServer.getInstance().startAutomaticCapture(camera);

    Gson gson = new GsonBuilder().create();

    camera.setConfigJson(gson.toJson(config.config));
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    if (config.streamConfig != null) {
      mjpegServer.setConfigJson(gson.toJson(config.streamConfig));
    }

    return camera;
  }

  // **************************************************************************
  // *
  // * Main Method
  // *
  // **************************************************************************
  public static void main(String... args) {
    if (args.length > 0) {
      configFile = args[0];
    }

    // Read configuration
    if (!readConfig()) {
      return;
    }

    // Start NetworkTables
    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();

    if (server) {
      System.out.println("Setting up NetworkTables server");
      ntinst.startServer();
    } else {
      System.out.println("Setting up NetworkTables client for team " + team);
      ntinst.startClientTeam(team);
    }

    // Start cameras
    List<VideoSource> cameras = new ArrayList<>();

    for (CameraConfig cameraConfig : cameraConfigs) {
      cameras.add(startCamera(cameraConfig));
    }

    // Hash map for distance calculations, these values were
    // collected empirically. The key is the actual distance in inches
    // from the front of the camera to the target. The value is
    // is the pixels per inch conversion rate at each distance. Knowing
    // that the actual distance between the two targets is 11 1/8 inches,
    // we can use this conversion rate to calculate the horizontal offset
    // at each distance.
    HashMap<Integer, Double> distanceHashMap = new HashMap<Integer, Double>();

    distanceHashMap.put(MIN_HASH_MAP_DISTANCE, 16.62921348);
    distanceHashMap.put(19, 16.0);
    distanceHashMap.put(20, 15.5505618);
    distanceHashMap.put(21, 14.29213483);
    distanceHashMap.put(22, 13.84269663);
    distanceHashMap.put(23, 13.21348315);
    distanceHashMap.put(24, 12.85393258);
    distanceHashMap.put(25, 12.49438202);
    distanceHashMap.put(26, 12.13483146);
    distanceHashMap.put(27, 11.7752809);
    distanceHashMap.put(28, 11.3258427);
    distanceHashMap.put(29, 10.96629213);
    distanceHashMap.put(30, 10.60674157);
    distanceHashMap.put(31, 10.33707865);
    distanceHashMap.put(32, 10.06741573);
    distanceHashMap.put(33, 9.707865169);
    distanceHashMap.put(34, 9.438202247);
    distanceHashMap.put(35, 9.078651685);
    distanceHashMap.put(36, 8.898876404);
    distanceHashMap.put(37, 8.719101124);
    distanceHashMap.put(38, 8.539325843);
    distanceHashMap.put(39, 8.269662921);
    distanceHashMap.put(40, 8.08988764);
    distanceHashMap.put(41, 7.91011236);
    distanceHashMap.put(42, 7.730337079);
    distanceHashMap.put(43, 7.550561798);
    distanceHashMap.put(44, 7.280898876);
    distanceHashMap.put(45, 7.191011236);
    distanceHashMap.put(46, 7.011235955);
    distanceHashMap.put(47, 6.921348315);
    distanceHashMap.put(MAX_HASH_MAP_DISTANCE, 6.741573034);

    NetworkTable networkTable = ntinst.getTable("datatable");

    NetworkTableEntry nteTargetingState = networkTable.getEntry("targState");
    NetworkTableEntry nteDistanceToTargetInInches = networkTable.getEntry("distTargetIn");
    NetworkTableEntry nteHorizontalOffsetToTargetInInches = networkTable.getEntry("horzOffToIn");

    CvSink cvSink = new CvSink("openCV Camera");

    Mat openCVOverlay = new Mat();

    // Start image processing on camera 0 if present
    if (cameras.size() >= 1) {

      // For OpenCV processing, you need a "source" which will be our camera and
      // a "sink" or "destination" which will be an ouputStream that is fed into 
      // an MJPEG Server. 
      // TODO - this will always get the first camera detected and that may be the back camera which is no bueno
      VideoSource frontCamera = cameras.get(0);

      cvSink.setSource(frontCamera);
      CvSource outputStream = new CvSource("2228_OpenCV", PixelFormat.kMJPEG, (int) IMAGE_WIDTH_PIXELS,
          (int) IMAGE_HEIGHT_PIXELS, DEFAULT_FRAME_RATE);

      // This is MJPEG server used to create an overlaid image of what the OpenCV processing is 
      // coming up with on top of the live streamed image from the robot's front camera.
      MjpegServer mjpegServer2 = new MjpegServer("serve_openCV", MJPEG_OPENCV_SERVER_PORT);
      mjpegServer2.setSource(outputStream);

      // Just some color constants for later use in drawing contour overlays and text
      Scalar greenColor = new Scalar(0.0, 255.0, 0.0);
      Scalar redColor = new Scalar(0.0, 0.0, 255.0);
      Scalar blueColor = new Scalar(255.0, 0.0, 0.0);
      Scalar blackColor = new Scalar(0.0, 0.0, 0.0);
      Scalar purpleColor = new Scalar(255.0, 0.0, 255.0);

      VisionThread visionThread = new VisionThread(frontCamera, new MyPipeline(), pipeline -> {

        // This grabs a snapshot of the live image currently being streamed
        cvSink.grabFrame(openCVOverlay);

        // Draw a vertical line down the center of the image (i.e., IMAGE_WIDTH / 2)
        Imgproc.line(openCVOverlay, new Point(IMAGE_WIDTH_PIXELS / 2, 25),
            new Point(IMAGE_WIDTH_PIXELS / 2, IMAGE_HEIGHT_PIXELS - 10), greenColor, 1, Core.LINE_4);

        // If, based on the OpenCV pipeline processing, we've found some filtered contours, let's
        // take a closer look at them. If not, just stay in the SEARCHING state.
        if (!pipeline.filterContoursOutput().isEmpty()) {

          // Overlay all the filtered contours onto the lived streamed image, this is too distracting
          // to leave in for competition but might be good for debug.
          //Imgproc.drawContours(openCVOverlay, pipeline.filterContoursOutput(), -1, blueColor);

          // Let's start out with an empty list of targets and insert ones into the list
          // that fit our criteria
          targets.clear();
          targetRects.clear();

          // We'll now loop though all the filtered contours provided by the OpenCV pipeline and
          // see if we can find some that match our critera.
          for (int contourIndex = 0; contourIndex < pipeline.filterContoursOutput().size(); contourIndex++) {

            // When examining each contour that the pipeline provides, we'll first get the bounding 
            // rectangle that encompases the contour. This rectangle is a vertical/horizontal 
            // rectangle around the object that is determined to be a contour.
            Rect rect = Imgproc.boundingRect(pipeline.filterContoursOutput().get(contourIndex));

            // Since the vision tape strips are angled, it is better to find the rotated rectangle that
            // better fits the shape of the tape strips, so we'll get that here.
            MatOfPoint2f newMtx = new MatOfPoint2f(pipeline.filterContoursOutput().get(contourIndex).toArray());
            RotatedRect rotatedRect = Imgproc.minAreaRect(newMtx);

            // The vision tape strips are rotated at specific angles on the game pieces so we'll get
            // the exact angle of the rotated rectangle for use in later analysis.
            double rectAspectRatio = (double) rotatedRect.size.height / (double) rotatedRect.size.width;
            double rotatedAngle = rotatedRect.angle;

            // In order to add a contour to the target list, the following conditions must be met:

            // 1. For low angle target strips (i.e., those around -15 degrees), the rotated
            //    rectangle's angle must be between TARGET_LOW_ANGLE_MIN_THRESHOLD and
            //    TARGET_LOW_ANGLE_MAX_THRESHOLD. Also for low angle target strips, the aspect 
            //    ratio be between TARGET_ASPECT_RATIO_MIN_THRESHOLD_FOR_LOW_ANGLE and
            //    TARGET_ASPECT_RATIO_MAX_THRESHOLD_FOR_LOW_ANGLE.

            // 2. For high angle target strips (i.e., those around -75 degrees), the rotated
            //    rectangle's angle must be between TARGET_HIGH_ANGLE_MIN_THRESHOLD and
            //    TARGET_HIGH_ANGLE_MAX_THRESHOLD. Also for high angle target strips, the aspect 
            //    ratio be between TARGET_ASPECT_RATIO_MIN_THRESHOLD_FOR_HIGH_ANGLE and
            //    TARGET_ASPECT_RATIO_MAX_THRESHOLD_FOR_HIGH_ANGLE.

            if ((rotatedAngle >= TARGET_LOW_ANGLE_MIN_THRESHOLD) && (rotatedAngle <= TARGET_LOW_ANGLE_MAX_THRESHOLD)) {

              if ((rectAspectRatio <= TARGET_ASPECT_RATIO_MAX_THRESHOLD_FOR_LOW_ANGLE)
                  && (rectAspectRatio >= TARGET_ASPECT_RATIO_MIN_THRESHOLD_FOR_LOW_ANGLE)) {

                targets.add(rect);
                targetRects.add(rotatedRect);

              }
            } else if ((rotatedAngle >= TARGET_HIGH_ANGLE_MIN_THRESHOLD)
                && (rotatedAngle <= TARGET_HIGH_ANGLE_MAX_THRESHOLD)) {

              if ((rectAspectRatio <= TARGET_ASPECT_RATIO_MAX_THRESHOLD_FOR_HIGH_ANGLE)
                  && (rectAspectRatio >= TARGET_ASPECT_RATIO_MIN_THRESHOLD_FOR_HIGH_ANGLE)) {

                targets.add(rect);
                targetRects.add(rotatedRect);

              }
            }
          }

          if (analyzeTargets()) {
            targetingState = TARGETING_STATE_ACQUIRING;
          }

          // Now that we think that we're looking at the right target (i.e., two correctly
          // angled vision tape strips with the right aspect ratio), we can move on with
          // with determining some distance calculations
          if ((targetingState == TARGETING_STATE_ACQUIRING) && (targets.size() == 2)) {

            // Display the ACQUIRING state text overlaid on the streaming image
            Imgproc.rectangle(openCVOverlay, new Point(0, 0), new Point(IMAGE_WIDTH_PIXELS - 2, 15), blackColor, -1);
            Imgproc.putText(openCVOverlay, "Acquiring Target", new Point(2.0, 10.0), Core.FONT_HERSHEY_SIMPLEX, 0.4,
                redColor, 1);
 
            // Get the bounding rectangles that encompass both targets
            Rect r1 = targets.get(0);
            Rect r2 = targets.get(1);

            // Determine the upper-left and lower-right points of the rectangle needed
            // to draw the image of the rectangle overlaid on the streaming image
            Point r1p1 = new Point(r1.x, r1.y);
            Point r1p2 = new Point(r1.x + r1.width, r1.y + r1.height);

            // Overlay the bounding rectangle onto image
            Imgproc.rectangle(openCVOverlay, r1p1, r1p2, blueColor, +1, 4);

            // Determine the "center X" value for the first target. This will be used
            // later on to calculate the distance, in pixels, between the two vision
            // tape strips.
            double contour1CenterXInPixels = r1.x + (r1.width / 2);

            // Determine the upper-left and lower-right points of the rectangle needed
            // to draw the image of the rectangle overlaid on the streaming image
            Point r2p1 = new Point(r2.x, r2.y);
            Point r2p2 = new Point(r2.x + r1.width, r2.y + r1.height);

            // Overlay bounding rectangle onto image
            Imgproc.rectangle(openCVOverlay, r2p1, r2p2, purpleColor, +1, 4);

            // Determine the "center X" value for the second target. This will be used
            // later on to calculate the distance, in pixels, between the two vision
            // tape strips.
            double contour2CenterXInPixels = r2.x + (r2.width / 2);

            // When far enough away from the side of the Cargo Ship, we can see two
            // full target strip pairs side by side. Vision was picking up on the
            // outer-most strips of each target as a valid pair so we need to set
            // a limit on how far away the strips can be in pixels. There's probably
            // a better way to do this based on distance to target.
            if (Math.abs(contour2CenterXInPixels - contour1CenterXInPixels) < MINIMUM_HORIZONTAL_OFFSET_REQ_IN_PIXELS) {

              // We need to determine how far each of the targets is away from the
              // center of the image. For now, this is measured in pixels but we'll
              // later be converting the distance to inches.
              double delta1 = 0.0;
              double delta2 = 0.0;

              delta1 = HALF_IMAGE_WIDTH_IN_PIXELS - contour1CenterXInPixels;
              delta2 = contour2CenterXInPixels - HALF_IMAGE_WIDTH_IN_PIXELS;

              // We want to use an average of the two rectangle heights to get a
              // better approximation of real target height's that we're seeing.
              // Again, this is still in pixels and we'll use it in the distance
              // calculation below.
              double avgPixelHeight = ((double) r1.height + (double) r2.height) / 2.0;

              // Distance Calculation:
              // distance = TargetHeightInFeet * YRes / (2 * PixelHeight *
              // tan(ViewAngleOfCamera))

              // The distance calculation now gives us something in a unit of measure (feet)
              // that we can use to provide guidance to the robot on where it is in relation
              // to
              // the target. We ultimately need distance to the target and how far off (left
              // to
              // right) we are from the center of the target.
              double calculatedDistanceToTargetInFeet = ((TARGET_HEIGHT_INCHES / 12.0) * IMAGE_HEIGHT_PIXELS)
                  / (2.0 * avgPixelHeight * CAMERA_FOV_ANGLE_CALC);

              // This is where things get a little 'hacky'. We determined through recording
              // the actual
              // distance between the front of the camera and the target at 1 inch intervals
              // from 18"
              // to 48" that the actual distance and calculated distance to the target using
              // the
              // equation above was consitently off by DISTANCE_CORRECTION_OFFSET inches, so
              // we're
              // subtracting that here. This is likely due to the CAMERA_FOV_ANGLE_CALC being
              // slightly
              // off but there is no reliable manufacturer data that gives us the FOV for the
              // Microsoft
              // HD cam that we're using.
              double correctedDistanceToTargetInInches = (calculatedDistanceToTargetInFeet * 12.0)
                  - DISTANCE_CORRECTION_OFFSET;

              // We'll now populate the network table with the corrected distance information
              nteDistanceToTargetInInches.setDouble(correctedDistanceToTargetInInches);

              // The next thing we want to do is to determine if the two vision tape strips
              // are in the horizontal center of the field of view.
              double horizontalOffsetInPixels = (delta2 - delta1) / 2.0;

              // To visually aid the driver, lets draw a center line overlaid on top of the
              // streaming
              // image of where we think the center of the target is. The goal would be to get
              // the
              // robot to close the gap between this line and the green line that shows the
              // center of
              // the field of view so that both lines align with each other. If we're not
              // perfectly
              // lined up, the red line could be on either side of the field of view's center
              // so we
              // have to take that into account.
              if (horizontalOffsetInPixels < 0.0) {

                Imgproc.line(openCVOverlay,
                    new Point(contour1CenterXInPixels
                        + (int) Math.round((contour2CenterXInPixels - contour1CenterXInPixels) / 2.0), 25),
                    new Point(
                        contour1CenterXInPixels
                            + (int) Math.round((contour2CenterXInPixels - contour1CenterXInPixels) / 2.0),
                        IMAGE_HEIGHT_PIXELS - 10),
                    redColor, 1, Core.LINE_4);

              } else if (horizontalOffsetInPixels > 0.0) {

                Imgproc.line(openCVOverlay,
                    new Point(contour2CenterXInPixels
                        - (int) Math.round((contour2CenterXInPixels - contour1CenterXInPixels) / 2.0), 25),
                    new Point(
                        contour2CenterXInPixels
                            - (int) Math.round((contour2CenterXInPixels - contour1CenterXInPixels) / 2.0),
                        IMAGE_HEIGHT_PIXELS - 10),
                    redColor, 1, Core.LINE_4);

              }

              // As we described above, we took measurements of a locked-in target at
              // distances
              // between 18" and 48" at one inch increments. The other data that we collected
              // at
              // each one inch increment, was the calculated distance, in pixels, between the
              // two
              // targets. Using this data, we built a "look-up table" that is indexed by an
              // integer
              // "key" from 18 to 48. This "key" represents the distance to target, rounded to
              // the
              // nearest inch from the camera. The "value" column in the table associated with
              // each
              // of these "keys" is a calculated "pixels per inch" value at that given
              // distance. We
              // can calculate this because we know the actual distance between the vision
              // tape strips
              // is 11 1/8 inches and we recorded the distance, in pixels, between the two
              // targets using
              // the OpenCV data provided above. This will give us a way to determine the
              // distance
              // in inches that we are off center at any distance from the target between 18
              // and 48
              // inches.

              // In order to get the "key" or row in the look-up table that we want, we'll
              // round
              // our corrected distance (which is a double) to target to an integer.
              int roundedDistanceToTargetInInches = (int) Math.round(correctedDistanceToTargetInInches);

              // Let's make sure the key value is valid to be looked up in the table (i.e, it
              // is
              // between 18 and 48 inches).
              if ((roundedDistanceToTargetInInches >= MIN_HASH_MAP_DISTANCE)
                  && (roundedDistanceToTargetInInches <= MAX_HASH_MAP_DISTANCE)) {

                // At this point we know that we've found two valid targets (right aspect ratio,
                // right angles) and that we're within our zone of 18 to 48 inches where we can
                // accurately calculate the horizontal offset distance. Since we know our
                // distance to target, we can look up the right value in the table to give us
                // our pixels per inch conversion rate to do our horizontal distance
                // calculation. We do have one wacky case where, at around 4 feet from the cargo
                // ship, we can see two cargo holes side by side and that gives us a valid
                // target orientation by seeing the left strip of the left-most hole and the
                // right strip of the right-most hole. We need to check for a minimum distance
                // before going into LOCKED>

                double horizontalOffsetInInches = horizontalOffsetInPixels
                    / distanceHashMap.get(roundedDistanceToTargetInInches);

                targetingState = TARGETING_STATE_LOCKED;

                // Let's populate the network table with the horizontal offset value
                nteHorizontalOffsetToTargetInInches.setDouble(horizontalOffsetInInches);

                // Like a number line, think of zero being perfectly centered. Any negative
                // values mean
                // that we're looking too far to the left and the real center is to the right. A
                // positive
                // number means that we're looking to0 far to the right and the real center is
                // to the left.
                if (horizontalOffsetInInches < 0.0) {
                  Imgproc.rectangle(openCVOverlay, new Point(0, 0), new Point(IMAGE_WIDTH_PIXELS - 2, 15), blackColor,
                      -1);
                  Imgproc.putText(openCVOverlay,
                      "Target locked @ " + roundedDistanceToTargetInInches + " in. away, "
                          + String.format("%.2f", Math.abs(horizontalOffsetInInches)) + " in. left of ctr",
                      new Point(2.0, 10.0), Core.FONT_HERSHEY_PLAIN, .7, greenColor, 1);

                } else if (horizontalOffsetInInches > 0.0) {
                  Imgproc.rectangle(openCVOverlay, new Point(0, 0), new Point(IMAGE_WIDTH_PIXELS - 2, 15), blackColor,
                      -1);
                  Imgproc.putText(openCVOverlay,
                      "Target locked @ " + roundedDistanceToTargetInInches + " in. away, "
                          + String.format("%.2f", Math.abs(horizontalOffsetInInches)) + " in. right of ctr",
                      new Point(2.0, 10.0), Core.FONT_HERSHEY_PLAIN, .7, greenColor, 1);

                } else {
                  Imgproc.rectangle(openCVOverlay, new Point(0, 0), new Point(IMAGE_WIDTH_PIXELS - 2, 15), blackColor,
                      -1);
                  Imgproc.putText(openCVOverlay,
                      "Target locked @ " + roundedDistanceToTargetInInches + " in. away and centered on target",
                      new Point(2.0, 10.0), Core.FONT_HERSHEY_PLAIN, .7, greenColor, 1);

                }
              }
            }
              
          } else {

            targetingState = TARGETING_STATE_SEARCHING;

            Imgproc.rectangle(openCVOverlay, new Point(0, 0), new Point(IMAGE_WIDTH_PIXELS - 2, 15), blackColor, -1);
            Imgproc.putText(openCVOverlay, "Searching...", new Point(2.0, 10.0), Core.FONT_HERSHEY_SIMPLEX, 0.4,
                redColor, 1);

          }
        } else {

          targetingState = TARGETING_STATE_SEARCHING;

          Imgproc.rectangle(openCVOverlay, new Point(0, 0), new Point(IMAGE_WIDTH_PIXELS - 2, 15), blackColor, -1);
          Imgproc.putText(openCVOverlay, "Searching...", new Point(2.0, 10.0), Core.FONT_HERSHEY_SIMPLEX, 0.4, redColor,
              1);

          System.out.println("No contours found that match filter criteria!!");
        }

        // Let's put the targeting state into the network table
        nteTargetingState.setDouble((double) targetingState);

        // This overlays all of the OpenCV stuff (bounding rectangles, text, etc.) over
        // the streaming image
        outputStream.putFrame(openCVOverlay);
      });

      // Start the thread's execution. Runs continuously until the program is terminated
      visionThread.start();

    } else {
      System.out.println("No cameras found");
    }

    // **************************************************************************
    // *
    // * Main "Forever" Loop
    // *
    // **************************************************************************
    for (;;) {
      try {
        Thread.sleep(10000);
      } catch (InterruptedException ex) {
        return;
      }
    }
  }
  
  private static boolean isTargetOrientationValid(RotatedRect rotatedRect1, RotatedRect rotatedRect2) {
    boolean isValid = true;

    // Case where targets are oriented like this: / /
    if ((isHighAngleTarget(rotatedRect1)) && (isHighAngleTarget(rotatedRect2))) {
      isValid = false;
    } else if ((isLowAngleTarget(rotatedRect1)) && (isLowAngleTarget(rotatedRect2))) {
      // Case where targets are oriented like this: \ \
      isValid = false;
    } else if ((isLowAngleTarget(rotatedRect1)) && (isHighAngleTarget(rotatedRect2))) {
      // Case where targets are oriented like this \ /
      isValid = false;
    }
    return isValid;
  }

  private static boolean isHighAngleTarget(RotatedRect rotatedRect) {
    boolean isHighAngleTarget = false;

    if ((rotatedRect.angle >= TARGET_HIGH_ANGLE_MIN_THRESHOLD)
        && (rotatedRect.angle <= TARGET_HIGH_ANGLE_MAX_THRESHOLD)) {
          isHighAngleTarget = true;
    }

    return isHighAngleTarget;
  }

  private static boolean isLowAngleTarget(RotatedRect rotatedRect) {
    boolean isLowAngleTarget = false;

    if ((rotatedRect.angle >= TARGET_LOW_ANGLE_MIN_THRESHOLD)
        && (rotatedRect.angle <= TARGET_LOW_ANGLE_MAX_THRESHOLD)) {
      isLowAngleTarget = true;
    }

    return isLowAngleTarget;
  }

  private static boolean analyzeTargets() {

    boolean validTargetPairFound = false;

    // We're only going to look at scenarios where
    // two, three or four targets have been found
    int numTargetsFound = targets.size();

    if ((numTargetsFound < 2)  || (numTargetsFound > 4)) {
      validTargetPairFound = false;
    } else {
      Rect rFirst = targets.get(0);
      Rect rLast = targets.get(numTargetsFound - 1);

      // If OpenCV has reversed the order of the rectangles
      // and put them in the array from right to left instead
      // of left to right, we'll just reverse the lists.
      if (rFirst.x > rLast.x) {
        Collections.reverse(targets);
        Collections.reverse(targetRects);
      }
    }
    
    if (numTargetsFound == 2) {

      Rect r1 = targets.get(0);
      Rect r2 = targets.get(1);

      RotatedRect rotRec1;
      RotatedRect rotRec2;

      if ((r1.x < (IMAGE_WIDTH_PIXELS / 2.0)) && (r2.x > (IMAGE_WIDTH_PIXELS / 2.0))) {

        rotRec1 = targetRects.get(0);
        rotRec2 = targetRects.get(1);

        validTargetPairFound = isTargetOrientationValid(rotRec1, rotRec2);

      } else {
        validTargetPairFound = false;
      }
    } else if (numTargetsFound == 3) {

      Rect r1 = targets.get(0);
      Rect r2 = targets.get(1);
      Rect r3 = targets.get(2);

      RotatedRect rotRec1;
      RotatedRect rotRec2;
      RotatedRect rotRec3;

      if ((r1.x < (IMAGE_WIDTH_PIXELS / 2.0)) && (r2.x > (IMAGE_WIDTH_PIXELS / 2.0))) {

        rotRec1 = targetRects.get(0);
        rotRec2 = targetRects.get(1);

        validTargetPairFound = isTargetOrientationValid(rotRec1, rotRec2);

        if (validTargetPairFound) {
          targets.remove(2);
          targetRects.remove(2);
        }

      } else if ((r2.x < (IMAGE_WIDTH_PIXELS / 2.0)) && (r3.x > (IMAGE_WIDTH_PIXELS / 2.0))) {
        rotRec2 = targetRects.get(1);
        rotRec3 = targetRects.get(2);

        validTargetPairFound = isTargetOrientationValid(rotRec2, rotRec3);

        if (validTargetPairFound) {
          targets.remove(0);
          targetRects.remove(0);
        }
      } else {
        validTargetPairFound = false;
      }

    } else if (numTargetsFound == 4) {
      Rect r1 = targets.get(0);
      Rect r2 = targets.get(1);
      Rect r3 = targets.get(2);
      Rect r4 = targets.get(3);

      RotatedRect rotRec1;
      RotatedRect rotRec2;
      RotatedRect rotRec3;
      RotatedRect rotRec4;

      if ((r1.x < (IMAGE_WIDTH_PIXELS / 2.0)) && (r2.x > (IMAGE_WIDTH_PIXELS / 2.0))) {

        rotRec1 = targetRects.get(0);
        rotRec2 = targetRects.get(1);

        validTargetPairFound = isTargetOrientationValid(rotRec1, rotRec2);

        if (validTargetPairFound) {
          targets.remove(3);
          targetRects.remove(3);
          targets.remove(2);
          targetRects.remove(2);
        }

      } else if ((r2.x < (IMAGE_WIDTH_PIXELS / 2.0)) && (r3.x > (IMAGE_WIDTH_PIXELS / 2.0))) {
        rotRec2 = targetRects.get(1);
        rotRec3 = targetRects.get(2);

        validTargetPairFound = isTargetOrientationValid(rotRec2, rotRec3);

        if (validTargetPairFound) {
          targets.remove(3);
          targetRects.remove(3);
          targets.remove(0);
          targetRects.remove(0);
        }
      } else if ((r3.x < (IMAGE_WIDTH_PIXELS / 2.0)) && (r4.x > (IMAGE_WIDTH_PIXELS / 2.0))) {
        rotRec3 = targetRects.get(2);
        rotRec4 = targetRects.get(3);

        validTargetPairFound = isTargetOrientationValid(rotRec3, rotRec4);

        if (validTargetPairFound) {
          targets.remove(1);
          targetRects.remove(1);
          targets.remove(0);
          targetRects.remove(0);

        } else {
          validTargetPairFound = false;
        }
      }
    }

    return validTargetPairFound;
  }
}
