```bash

cd ros
rosdep update && rosdep install -r -y -i --from-paths src --rosdistro kinetic

catkin_make
```