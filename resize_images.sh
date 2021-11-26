#!/bin/bash
for file in rgb_*; do convert $file -resize 640x480! res-$file; done