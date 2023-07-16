cd /home/snowden/workplace/state_estimation_for_robotics/state_estimation/build
rm -r ./*
cmake ..
make 
./LG_KF_APP
python /home/snowden/workplace/state_estimation_for_robotics/state_estimation/visual/show_state_line.py
xdg-open /home/snowden/workplace/state_estimation_for_robotics/state_estimation/build/my_figure.png