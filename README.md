SRF 2024 code for Crescendo Season

First commit includes only SRF's 2024 swerve drivebase, adapted from 
Team 364's BaseFalconSwerve, but heavily modified (and hopefully
simplified) to support just SDS Mk4 and Mk4i modules, specifically using NEO motors
for steering, Falcon500 motors for drive, and the Pigeon2 for gyro. 
Conversion methods (to and from Motor encoder units) have been replaced by simple conversion factors defined in constants.java for efficiency.
Overall swerve and individual module data are now published to a single custom Shuffleboard tab. However, the data to that tab sometimes does not always update at run time - cause not yet known, and even when it does appear the list order is never the same as the order used for list creation. Fortunately the order can be manually rearranged, but unfortunately once done, the new arrangement cannot be saved, since WPILIB does not support saving any layouts for "program created" Tabs. Catch 22 at the moment.
One other curious bug observed but not yet tracked down - when using translate and strafe joystick inputs simultaneously, the travel speed of the robot increases proportional to the amount of each (doubling when present in equal amounts) as compared to the robot speed when just translate, or just strafe, are present. Possibly introduced when creating ChassisSpeeds, or when converting toSwerveModuleStates?)
Be sure to check TODO changes, especially in constants.java.