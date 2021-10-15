# Drawing input file

The drawing input files come from [here](https://github.com/daeunsong/LindeBuzoGrayStippling/tree/tsp). Make sure you install all the [dependencies](https://github.com/daeunsong/LindeBuzoGrayStippling/tree/tsp) and follow the [build instructions](https://github.com/daeunSong/LindeBuzoGrayStippling/blob/tsp/README.md#building).

## Format
```shell
width height
x1 y1
x2 y2
... 
```
The first line of the drawing input file contains the **width** and **height** of the original input image. 

Starting from the second line, *(x, y)* coordinates are listed. This coordinates are normalized in [0, 1]. Therefore, you must multiply the target width and height of the drawing to get the proper sized darwing as you expected.   

(x, y)_drawing -> (y, z)_robot
*** add figure

For convenience, we set the TARGET SIZE as a target height. So following should be done:

```sh
double ratio = width / height;
y = (-stod(tempSplit[0])+0.5) * ratio * TARGET_SIZE;
z = (-stod(tempSplit[1])+0.5) * TARGET_SIZE;
```

This is done automatically for you already in [here](https://github.com/daeunSong/large_scale_drawing/blob/6989aee903cb2b19dad5aec67036164c4c4fce18/iiwa/src/drawing_input.cpp#L63). If you wish to change the target size, just change the value in [here](https://github.com/daeunSong/large_scale_drawing/blob/6989aee903cb2b19dad5aec67036164c4c4fce18/iiwa/include/drawing_input.h#L30).


### Flat Surface

*** add figure

Now this is transformed into a 3D space. *x* is determined as a detected wall pose, which should be sent when reading the drawing input.


### Curved Surface



