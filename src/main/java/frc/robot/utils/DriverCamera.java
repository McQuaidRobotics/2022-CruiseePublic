// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.utils;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.HttpCamera;
// import edu.wpi.first.cscore.MjpegServer;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import frc.robot.constants.kVision;

// /** Add your docs here. */
// public class DriverCamera {
//     private MjpegServer server;
//     private HttpCamera LLFeed;
//     private UsbCamera cargoCam;
//     private int cameraStream = 0;

//     public DriverCamera(){
//         ShuffleboardTab dashboardTab = Shuffleboard.getTab("Dash");
//         LLFeed = new HttpCamera("driverCam", )kVision.DRIVER_CAM_URL);
//         cargoCam = CameraServer.startAutomaticCapture(0);
//         cargoCam.setConnectVerbose(0);
//         server = CameraServer.("Toggle Cam");
//         server.setSource(LLFeed);
//         dashboardTab.add(server.getSource()).withWidget(BuiltInWidgets.kCameraStream).withPosition(1, 1).withSize(5, 4)
//             .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));// specify widget properties here
//         Shuffleboard.selectTab("Dash");
//         ShuffleboardTab mVisionTab = Shuffleboard.getTab("Vision");
//         mLLX = mVisionTab.add("Limelight X", 0.0).getEntry();
//         mDist = mVisionTab.add("Limelight Dist", 0.0).getEntry();
//         mArea = mVisionTab.add("Area", 0.0).getEntry();
//         mLimeLight = new LimeLight();
//         configLimelightVision();
//         mPixy = new MkPixy();
//     }
// }
