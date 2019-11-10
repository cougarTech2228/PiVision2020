# PiVision2020
Raspberry Pi and GRIP generated vision code

## Building on Desktop

Java 11 is required to build.  Set your path and/or JAVA_HOME environment
variable appropriately.

1) Run "./gradlew build" from a VS Code terminal

## Deploying from Desktop

On the rPi web dashboard:

1) Make the rPi writable by selecting the "Writable" tab
2) In the rPi web dashboard Application tab, select the "Uploaded Java jar"
   option for Application
3) Click "Browse..." and select the "piGrip2020-all.jar" file in
   your desktop project directory in the build/libs subdirectory
4) Click Save

The application will be automatically started.  Console output can be seen by
enabling console output in the Vision Status tab.