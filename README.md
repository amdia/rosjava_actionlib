# Actionlib for Rosjava
A pure java implementation of [actionlib](http://wiki.ros.org/actionlib) for [rosjava](http://wiki.ros.org/rosjava).

## Key Features:
1. Easy developing and testing in any OS (Windows, Linux, Mac) without a ROS installation. 
2. Compatible and usable with [ROS](https://www.ros.org). Tested with [ROS Indigo](http://wiki.ros.org/indigo).

## Requirements:
* Java 1.8 or greater (OpenJDK should work). Developed in JDK 11.
* For usage with a ROS installation the following packages are needed: ```ros-indigo-actionlib``` ```ros-indigo-actionlib-tutorials``` ```ros-indigo-genjava```
 

## Downloading:
1. Download the latest release of the project: https://github.com/ernestmc/rosjava_actionlib/archive/v0.2.0.zip
2. Unzip it somewhere.

## Compiling:
1. TODO

## Running a test client:
1. TODO

## Output from the test client
The test client will connect to the fibonacci server and send it a goal. It
should then receive feedback from the server and a final response. The output
should look something like this:
```
Loading node class: com.github.rosjava_actionlib.SimpleClient

Waiting for action server to start...
Action server started.

Sending goal...
Sent goal with ID: /fibonacci_test_client-1-1453494018.17000000
Waiting for goal to complete...
Feedback from Fibonacci server: 0 1 1
Feedback from Fibonacci server: 0 1 1 2
Feedback from Fibonacci server: 0 1 1 2 3
Got Fibonacci result sequence: 0 1 1 2 3
Goal completed!

Sending a new goal...
Sent goal with ID: /fibonacci_test_client-2-1453494021.25000000
Cancelling this goal...
Feedback from Fibonacci server: 0 1 1
Got Fibonacci result sequence:
Goal cancelled succesfully.

Bye!
```

## Running a test server:
1. TODO

## Output from the test server
The test server will start running and wait for clients to connect and send goal messages.
Once the fibonacci client sends a goal, the server accepts it and sends a result. The output
should look something like this:
```
Goal received.
Goal accepted.
Sending result...
```

## Running demos for the server and the client
You can launch a demo client and a fibonacci action server all at once using:
```
roslaunch rosjava_actionlib client_demo.launch --screen
```


You can also launch a demo server and a fibonacci action client all at once using:
```
roslaunch rosjava_actionlib server_demo.launch --screen
```

Use Ctrl+C to stop the execution once it's finished.


## Running unit tests
```
$ cd src/rosjava_actionlib/
$ ./gradlew test
```
