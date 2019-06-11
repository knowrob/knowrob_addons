set terminal pdf
set output "/home/ros/user_data/heatmap_mixture.pdf"

set datafile separator ","

set style fill solid
set style fill solid border -1
set xtic scale 0

set key off

set xlabel "X"
set ylabel "Y"

set title "Gaussian Mixture Distribution Heatmap"

set palette defined (0 "blue", 17 "#00ffff", 33 "white", 50 "yellow", 66 "red", 100 "#990000", 101 "grey")

plot "/home/ros/user_data/data_mixture.csv" using 1:2:3 w image
