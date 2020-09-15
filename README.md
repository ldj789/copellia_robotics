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
