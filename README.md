## Coppelia Robotics
This project is a starting guide to getting Copellia running for robotics simulation with Python. We curate helpful resources and share our solutions as we get Copellia running on windows and osx.

### Installation

```Make sure to have the server side running in CoppeliaSim: 
in a child script of a CoppeliaSim scene, add following command
to be executed just once, at simulation start:

simRemoteApi.start(19999)

then start simulation, and run this program.

IMPORTANT: for each successful call to simxStart, there
should be a corresponding call to simxFinish at the end!
```

### Resources
- [python-api](https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm) - Coppelia Python docs
- [youtube/tutorial](https://www.youtube.com/playlist?list=PLjzuoBhdtaXOoqkJUqhYQletLLnJP8vjZ) - Youtube playlist



### Connect Coppelia with Python:

**Requirement**:  

Python Version: 3.7.4, 64 bit
Platform used: PyCharm/Spyder


**Instruction**:
1. Open Coppelia
2. python_vrep_mazi_path.ttt
2. Disable Robot Non-threaded child script (here use robot Pioneer_p3dx)
2. Create a new directory
3. Copy File to new directory : C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\remoteApiBindings\python\python 
4. Copy File to new directory:  C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\remoteApiBindings\lib\lib\Windows\remoteApi.dll
5. open simpleTest.py to get test run


**Functions to Lookup**:
https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm


### Example   python_vrep_mazi_path.ttt:

