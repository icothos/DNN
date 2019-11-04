# Shortest Path Simple Polygon
(the citation thing) Optimal Shortest Path Queries in a Simple Polygon 

### Finding the Shortest Path in a Simple Polygon
This is a simple program to compute and visualize the shortest path between two points in a simple polygon. Some input files are provided in the input directory by default. The user may add her own set of input polygons by following the instructions below.
The computation process involves polygon decomposition (triangulation) and concatenation of specifically designed data structures ('Funnels') to facilitate the computation.
### Testing the Code
1. Once you start the program, you will be asked to press **'1'**  to add a input polygon and **'2'** to actually test out the algorithm.
2. **Adding a new input polygon:** In counter clock-wise direction, draw your polygon by clicking within the given screen area. Press the upper arrow key if you wish to save your polygon. Your polygon will be saved under the name "input/new_input_please_save_separately.txt". Unless you rename your file (check the format below), it will be overwritten once a new polygon is saved. You can start over by pressing the down arrow key, and undo a single point by pressing the left arrow key.
3. **Finding the Shortest Path:** Type in a valid integer number 'X' if you wish to open file 'inputX.txt'. Select two points within the simple polygon and you will see the shortest path appear on the screen. Press the space bar to clear your test inputs and choose another set.

### Development Enviroment
Visual Studio 2017, OpenGL 4.6
