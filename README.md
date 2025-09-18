# 6914 Charging Chaos Into The Deep Season Code
All code is java files. Only the java files are in here this is not everything need for a FTC robot. Only final versions will be commented.

## Code List

### TeleOp
- ***[telOpV1](TeleOp/telOpV1.java)***
  - This is version 1 of the teleOp, LimeLight on back of robot for AprilTag location and picking Specimen off wall.
- ***[telOpV2](TeleOp/telOpV2.java)***
  - This is the final version, Limelight mounted above intake for auto alignment.
- ***[judging](TeleOp/judging.java)***
  - This was for during judging to be able to display the lights and there meaning.

### Autonomous
- ***[BlueBlueV1](Autonomous/BlueBlueV1.java)***
  - This is for blue SPECIMEN auto.
  - clips SPECIMEN on HIGH CHAMBER | pushes one SAMPLE into OBSERVATION ZONE at the same time picks SPECIMEN off wall | clips SPECIMEN on HIGH CHAMBER | picks SPECIMEN off wall from in OBSERVATION ZONE | parks in OBSERVATION ZONE by extending intake
- ***[BlueYellowV1](Autonomous/BlueYellowV1.java)***
  - First version with failsafes.
  - place SAMPLE in HIGH BASKET | pick up SAMPLE off inside SPIKE MARK | place SAMPLE in HIGH BASKET | pick up SAMPLE off middle SPIKE MARK | place SAMPLE in HIGH BASKET | pick up SAMPLE off wall SPIKE MARK | place SAMPLE in HIGH BASKET
- ***[BlueYellowV2](Autonomous/BlueYellowV2.java)***
  - First version with lift and chassis movement.
  - place SAMPLE in HIGH BASKET | pick up SAMPLE off inside SPIKE MARK | place SAMPLE in HIGH BASKET | pick up SAMPLE off middle SPIKE MARK | place SAMPLE in HIGH BASKET | pick up SAMPLE off wall SPIKE MARK | place SAMPLE in HIGH BASKET | park touching LOW RUNG
- ***[BlueYellowV3](Autonomous/BlueYellowV3.java)***
  - Final Version && first version with Limelight alignment.
  - place SAMPLE in HIGH BASKET | pick up SAMPLE off middle SPIKE MARK | place SAMPLE in HIGH BASKET | pick up SAMPLE off inside SPIKE MARK | place SAMPLE in HIGH BASKET | pick up SAMPLE off wall SPIKE MARK | place SAMPLE in HIGH BASKET | park touching LOW RUNG
- ***[RedRedV1](Autonomous/RedRedV1.java)***
  - This is for red SPECIMEN auto.
  - clips SPECIMEN on HIGH CHAMBER | pushes one SAMPLE into OBSERVATION ZONE at the same time picks SPECIMEN off wall | clips SPECIMEN on HIGH CHAMBER | picks SPECIMEN off wall from in OBSERVATION ZONE | parks in OBSERVATION ZONE by extending intake
- ***[RedYellowV1](Autonomous/RedYellowV1.java)***
  - First version with failsafes.
  - place SAMPLE in HIGH BASKET | pick up SAMPLE off inside SPIKE MARK | place SAMPLE in HIGH BASKET | pick up SAMPLE off middle SPIKE MARK | place SAMPLE in HIGH BASKET | pick up SAMPLE off wall SPIKE MARK | place SAMPLE in HIGH BASKET
- ***[RedYellowV2](Autonomous/RedYellowV2.java)***
  - First version with lift and chassis movement.
  - place SAMPLE in HIGH BASKET | pick up SAMPLE off inside SPIKE MARK | place SAMPLE in HIGH BASKET | pick up SAMPLE off middle SPIKE MARK | place SAMPLE in HIGH BASKET | pick up SAMPLE off wall SPIKE MARK | place SAMPLE in HIGH BASKET | park touching LOW RUNG
- ***[RedYellowV3](Autonomous/RedYellowV3.java)***
  - Final Version && first version with Limelight alignment.
  - place SAMPLE in HIGH BASKET | pick up SAMPLE off middle SPIKE MARK | place SAMPLE in HIGH BASKET | pick up SAMPLE off inside SPIKE MARK | place SAMPLE in HIGH BASKET | pick up SAMPLE off wall SPIKE MARK | place SAMPLE in HIGH BASKET | park touching LOW RUNG

### Other
- ***[PoseStorage](Other/PoseStorage.java)***
  - This is for storing "*Position*" variables that where save as long as the robot is running. This saves the data in the move from auto to teleOp and if the robot disconnects. Stored data like Position, Aliance Color, Scoring Type, and more.
- ***[Hardware](Other/Hardware.java)***
  - This is for setting all of the motors, servos, and sensor.
- ***[MecanumDrive](Other/MecanumDrive.java)***
  - This was not writen by me. This is from Road Runner.
