## Coppelia Robotics
This project is a starting guide to getting Copellia running for robotics simulation with Python. We curate helpful resources and share our solutions as we get Copellia running on windows and osx.



### Installation

- [Version: CoppeliaSim_Release 4.0.0_edu](https://www.coppeliarobotics.com/previousVersions)
- Click on the link and download the file
- Open the filer (folder) you just downloaded and click on the icon CoppeliaSim:

  ![alt text](https://niryo.com/wp-content/uploads/2019/12/CoppeliaSim.png)
  
 - If you can load the program, then you are good to go.
 - If not (for Mac), try this on your terminal and rerun the file. 
    > - cd yourCoppeliasimDirectory
    > - sudo xattr -r -d com.apple.quarantine *

### Set up

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
- [CoppeliaSim User Manual](https://www.coppeliarobotics.com/helpFiles/index.html)



### Connect Coppelia with Python:

**Requirement**:  

Python Version: 3.7.4, 64 bit
Platform used: PyCharm/Spyder


**Instruction**:
```
1. Open Coppelia
2. python_vrep_mazi_path.ttt
2. Disable Robot Non-threaded child script (here use robot Pioneer_p3dx)
2. Create a new directory
3. Copy File to new directory : C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\remoteApiBindings\python\python 
4. Copy File to new directory:  C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\remoteApiBindings\lib\lib\Windows\remoteApi.dll
5. open simpleTest.py to get test run
```


### Example   python_vrep_mazi_path.ttt:

