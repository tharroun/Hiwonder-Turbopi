# Hiwonder Turbopi
Exploring activities with the Hiwonder Turbopi

This is just me having fun learning some vision and robotics.


## SDK

This folder contains the minimal sdk for communicating with the expansion board and a few untility files.

The TurboPi version I have has the following features, which can be controlled by these files.

`tah_sdk_board.py`
- 4 motors
- 2 servo
- 1 led
- battery voltage

`hw_sdk_infrared.py`
- 4 IR readers

`hw_sdk_sonar.py`
- the front facing sonar

The motor control is based on [Dima Berezovskyi's](https://github.com/dmberezovskyii/fast-hiwonder/tree/main) code, to which I added the battery check and servo control. The sonar and infrared control are pretty much unchanged from Hiwonder.

The `tah_sdk_mecanum.py` changes some of the math from Hiwonder, such that 0 degrees orientation is forward faceing, and proceeds anti-clockwise from there.

## Color and Shape calibration

Some experimenting with OpenCV. The TurboPi camera is only 640x480, and the color is very subject to lighting conditions. With a broad definition of a particular color, it works okay.

## Color following

I wrote two equivlent versions of a color-following code, one using threading, and the other multiprocessing.

- I need to speed-up the servo motion, but it does follow a slow-moving object.
- The car rotates as the servo reaches it's limits of horizonatal movement to keep the object in frame.
- The car moves forward, backward to maintain constant sized object.
- If the car needs to move forward or backward while the camera is looking far roght or left, the car moves at a more diagonal angle.
- Image processing, camera servos control, and motors control each run in their own thread (or process).

## Threading and multiprocess

Some experiments with flow of control and proper clean up with python `threading` or `multiprocesses`. 