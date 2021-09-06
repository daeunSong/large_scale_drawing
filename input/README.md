#Drawing input file

## Format
```shell
width height
x1 y1
x2 y2
... 
```
The first line of the drawing input file contains the width and height of the original input image. 

Starting from the second line, (x, y) coordinates are listed. This coordinates are normalized in [0, 1]. Therefore, you must multiply the target size of the drawing when performing the drawing.
