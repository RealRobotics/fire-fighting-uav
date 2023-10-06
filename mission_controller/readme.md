# Mission controller 

The mission controller utilises [behaviour trees](https://en.wikipedia.org/wiki/Behavior_tree_(artificial_intelligence,_robotics_and_control)#:~:text=Article%20Talk,tasks%20in%20a%20modular%20fashion.). 
In particular, it is based on c++ library called [BehaviorTree.Cpp](https://www.behaviortree.dev/). 
Before continuing to read the explanation below, please familiarise yourself with the semantics of behaviour trees.

# Supported scenario
One of the big adventages of behaviour trees is that they are easily configurable by a text file, which means the code can stay same for different scenarios.
There is a folder **config** where you will find xml files, including example of behaviour trees.

## Monitoring scenario

A UAV needs to patrol an area which is specified by a list of waypoints described by their GPS coordinates. If the UAV reaches the last waypoint from the list, it flies to the first one and keeps repeating until an end condition occurs. There are two sources for the end condition to occur: either a command came from a ground station or the UAV’s battery is low.
In the former case, the UAV returns to the position where it started, e.g. to its home location. 
In the latter case, we distinguish between two levels for low battery: it is either mission critical or safety critical. If the UAV’s battery produces mission critical status, then the UAV returns to its home location. Finally, if the battery status is safety critical,  the UAV lands immediately. 

This scenario is supported by the **monitoring_scenario.xml**

### Known limitations (as of 06/10/2023)
- the drone will perform "star" pattern on waypoints; this mean that the drone will fly from its home location to the wp and then return to its home location before flying to another wp. This is due limitations of dji_sdk library which doesn't allow to fly to a single wp but requires a set of minimal three of them, when the first and the last needs to be home locations
- it is assumed in this file, that actions of the drone cannot be interrupted (this is due limitations of FCS interface). You can check **interuptable_monitoring_scenario.xml** for an example of an extended tree which would handle if actions are interruptable. However, this file hasn't been tested so it may contain mistakes.

### Scenario tree explanation

The main behaviour of this case study is a sequence of four steps: 
![Four main steps of scenario](img/mission_behaviour_1.png)


After each step SUCCEESed, the next step is triggered to run.

Next, we can add the start and stop signals from the ground station:


Notice, we have a new first step. This step consists of a decorator and a condition node (please refer to behaviour tree terminology if you are not sure about the meaning). The condition checks if the ground station requested start. It can return SUCCESS or FAILURE. The decorator above it will produce status RUNNING until the condition returns SUCCESS. As a result, the other nodes won’t be triggered until then. 

The step fly changed too. It now consists of a reactive fallback, a condition and an action node. The working of there three nodes is as follows. First, the reactive fallbacks get triggered. Then, it triggers the condition “Has the ground station requested a stop?”. The outcome of a condition node is either SUCCESS or FAILURE. 
In case of a SUCCESS, the reactive fallback will be SUCCESS too and the action node land will be triggered next.
In case of FAILURE, the reactive fallback will trigger the action node Fly. Action nodes can return RUNNING, SUCCESS and FAILURE. 
In case of RUNNING, the reactive fallback node will trigger again the first condition node
In case of SUCCESS, the reactive fallback node returns SUCCESS too and the next action node Land is triggered
In case of FAILURE, the reactive fallback node returns FAILURE which gets propagated to the top node which returns FAILURE too and the whole scenario has failed.

Now we can focus on the action Fly. It can be modelled as a subtree. We can start with a very simple tree:


This calls the action Fly to WP (=waypoint) n times, where n is the number of waypoints. But we said, we want to keep repeating until the ground station requests a stop or the battery is low. Hence, the new subtree considering the ground station would look as follows. 

We utilise a new decorator Keep Running Until Failure, which ensures that the action Fly to WP is called repeatedly. This tree will result in SUCCESS only when the condition Has ground station requested stop? will result in SUCCESS.

However, we need to introduce how to pass a WP to the action Fly to WP. This is via ports. 

