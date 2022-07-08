# *Step 1*  clone:
  #please put this project to home path, as "~/"

  cd ~
  
  git clone https://github.com/karasi-D/warch_planner.git
# *Step 2*  compile:
  cd ~/warch_planner
  
  code src/
  
  #1.find uav_simulator/map_server/map_generator/random_complex_generator.cpp 
  
  #2.change line41 to your path: only need change "karasi" to your username "?"
  
  cd src/
  
  catkin_init_workspace
  
  cd ..
  
  catkin_make -j4
# *Step 3*  run:
  
  #In a terminal at the warch_planner/ folder, run the planner in simulation
  
  source devel/setup.bash
  
  roslaunch warch_plan run_in_single.launch
