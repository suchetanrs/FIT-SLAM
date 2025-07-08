# Gnuplot script to visualize the A* pathfinding algorithm

set terminal pngcairo size 800,600 enhanced font 'Verdana,12'
set output 'pathfinding.png'

set title "A* Pathfinding Visualization"
set xlabel "X-axis"
set ylabel "Y-axis"
set xrange [-1:10]
set yrange [-1:6]
set grid

# Draw grid lines
set style line 1 linecolor rgb '#dcdcdc' linetype 1 linewidth 1
set for [i=0:10] arrow from i,0 to i,5 nohead linestyle 1
set for [j=0:5] arrow from 0,j to 9,j nohead linestyle 1

# Plot obstacles, path, start, and goal points
plot 'grid.dat' using 1:2 with points pointtype 7 pointsize 1.5 lc rgb 'red' title 'Obstacles', \
     'path.dat' using 1:2 with linespoints pointtype 7 pointsize 1 lc rgb 'blue' lw 2 title 'Path', \
     '-' using 1:2:3 with labels font ',14' notitle, \
     '-' using 1:2 with points pointtype 7 pointsize 1.5 lc rgb 'green' title 'Start', \
     '-' using 1:2 with points pointtype 7 pointsize 1.5 lc rgb 'purple' title 'Goal'

# Data for labels (start and goal points)
0 0 "Start"
4 9 "Goal"
e

# Data for start point
0 0
e

# Data for goal point
4 9
e
