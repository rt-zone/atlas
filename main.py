from atlas import Atlas
import time

a = Atlas()
a.setMoveSpeed(30)   # % as in your original code

# start moving forward in background thread
a.moveForward()

# do other things, sleep, etc.
time.sleep(5)

# request stop
a.stopMove(wait=True)   # wait=True blocks until the motion loop exits
