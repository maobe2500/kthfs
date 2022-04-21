# Exercise 1
## Dependencies:
The only dependency for exercise 1 is ros and rospy

## How to use
- Use catkin build to initialize the workspace. 
	```
	cd kthfs
	catkin build
	```
- Source the setup.bash file from the devel folder 
	```
	source devel/setup.bash
	```
- Start roscore in another terminal.
	```
	roscore
	```
- Use the lanch file in package2 to start the subscriber node and publisher node
	```
	roslaunch package2 package2_subscriber_node.launch
	```

After the last step you should see this being printed to the terminal
```
[INFO] [1650568812.225029]: 352
[INFO] [1650568812.226927]: publishing 2346.66666667 to /kthfs/result
```

# Exercise 2
## Dependencies:
#### Numpy
Use
```
pip3 install numpy
```
Or visit https://numpy.org/install/ for installation instructions
#### Matplotlib
Use 
```
pip3 install matplotlib
```
Or visit https://matplotlib.org/stable/ for installation instructions
#### Tkinter
Is included in standard python library. If not use 
```
pip3 install tk
```

## How to use
Open a terminal in exec2 folder and run
```
chmod +x ./GUI_Plotter.py
./GUI_Plotter.py
```
