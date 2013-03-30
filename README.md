Ball tracking with particle filter on color.

workflow:
* Segment ball from scene (with simple PassThough filter) - DONE
* Convert filtered cloud from RGBA to HSV space - DONE
* Calculate hue histogram
* Make PF with constant position motion model

