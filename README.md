### Dependencies

- OpenCV
- popt

### Compilation

Compile all the files using the following commands.

```bash
mkdir build && cd build
cmake ..
make
```

Make sure your are in the `build` folder to run the executables.

### Data

Some sample calibration images are stored in the `snapshots/pack2` folder.

### Running calibration

Run the executable with the following command from the 'build' directory

```bash
./fisheye_calibration -d [img_dir] -r [board_rows] -h [board_cols] -s [square_size]
```

For example if you use the images in the `snapshots/pack2` folder run the following command

```bash
./fisheye_calibration -i ../snapshots/pack2/ -r 10 -c 7 -s 25
```
